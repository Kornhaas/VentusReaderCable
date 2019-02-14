/*

   VentusReaderCable
  
   Sketch reads values from the Ventus W132, W174 weather station sensors via cable.
   Original with 433.92 MHz, but it's the same principle

   I'm a beginner, the sketch can contain errors and my english is terrible!

   TODO: error handling

   Hardware
   nodemcu lolin v3
   https://www.reichelt.de/ersatz-windmesser-fuer-funkwetterstation-ventus-w132-p132595.html
   https://www.reichelt.de/ersatz-regenmesser-fuer-ventus-funkwetterstation-ventus-w174-p132594.html

   credits and inspirations
   http://forum.arduino.cc/index.php?topic=136836.15 #19 code from user "jurs"
   https://github.com/yu55/auriol_reader
   http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf
   https://glsk.net/2018/02/battery-powered-weather-station-with-esp8266-and-bme280/

*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config.h"

// sensor connects
#define W132_DATAPIN 5 // D1/GPIO5 connected data pin of W132 module
#define W174_DATAPIN 4 // D2/GPIO4 connected data pin of W174 module

// ventus message details
#define VENTUS_SYNC 9000  // length in µs of starting pulse
#define VENTUS_ONE 4000   // length in µs of ONE pulse
#define VENTUS_ZERO 2000  // length in µs of ZERO pulse
#define VENTUS_TOL 400    // pulse length variation in µs for ONE and ZERO pulses
#define VENTUS_DATALEN 32 // Ventus message data lenght
#define VENTUS_CRCLEN 4   // Ventus message CRC lenght

// possible Errors, todo: Error handling
#define VENTUS_ERROR_CRC 1          // W132/W174 value CRC invalid
#define W132_ERROR_WINDDIRECTION 2  // value wind direction invalid
#define W132_ERROR_TEMPERATURE 3    // value temperature invalid
#define W132_ERROR_HUNIDITY 4       // value humidity invalid

// reveived bits
#define W132_RECEIVED_TEMPHUM 0     // bit 0 is set = received temperature & humidity
#define W132_RECEIVED_WINDSPEED 1   // bit 1 is set = received wind speed
#define W132_RECEIVED_WINDDIRGUST 2 // bit 2 is set = received wind direction & gust
#define W174_RECEIVED_RAIN 3        // bit 3 is set = received rain

// on/off 
#define WITH_BME280 1         // 0 to disable WITH_BME280 sensor readout.
#define WITH_DEEPSLEEP 1      // 0 to disable deep sleep
#define WITH_MQTT 1           // 0 to disable MQTT
#define WITH_CONFIGINTERN 0   // 0 to set external WLAN/MQTT configs
#define WITH_DEBUG 1          // 0 to disable debug output //TODO DEBUG_LEVEL
#define WITH_DEBUG_MQTT 1     // 0 to disable MQTT debug output 
#define WITH_DEBUG_SENSORS 0  // 0 to disable Sensor debug output

// debug functions
#if WITH_DEBUG > 0 || WITH_DEBUG_MQTT > 0 || WITH_DEBUG_SENSORS > 0
  #define SERIALSPEED 74880 // Set speed of serial in Arduino IDE
  #define debugStart(...) Serial.begin(__VA_ARGS__)   //debugStart is a macro, begin
  #define debug(...)      Serial.print(__VA_ARGS__)   //debug is a macro, debug print
  #define debugln(...)    Serial.println(__VA_ARGS__) //debugln is a macro, debug print print with LF
#else
  #define debugStart(...) //now defines a blank line
  #define debug(...)      //now defines a blank line
  #define debugln(...)    //now defines a blank line
#endif

// duration deep sleep/delay
#define SLEEP_S 25  // xx secs. sleep

// others
#define FIFOSIZE 8 // fifo buffer size
#define SEALEVELPRESSURE_HPA (1013.25) // for BME280 to calculate approx. altitude

#if WITH_MQTT > 0
  /************************* WiFi Access Point ****************************/
  #if WITH_CONFIGINTERN > 0 // if not defined in config.h
  #define WLAN_SSID       "SSID"
  #define WLAN_PASSWORD   "PASSWORD"
  #endif
  /************************* MQTT Server *********************************/

  #if WITH_CONFIGINTERN > 0 // if not defined in config.h
  #define MQTT_SERVER      "Broker IP or Hostname"
  #define MQTT_SERVERPORT  1883                   // use 8883 for SSL
  #define MQTT_USERNAME    "USERNAME"
  #define MQTT_PASSWORD    "PASSWORD"
  #endif
  
  //changed in PubSubClient.h packed size from 128 to 320
  //MQTT_MAX_PACKET_SIZE 320
  
  /************************* MQTT Feeds *********************************/
  const char* sensorTopic = "/home/sensors";
  const char* deviceTopic = "/home/devices"; //TODO
  
  // ESP8266 WiFiClient class to connect to the MQTT server.
  WiFiClient WiFiClient;
  // or... use WiFiFlientSecure for SSL
  //WiFiClientSecure WiFiClient;
  
  // MQTT client class
  PubSubClient client(WiFiClient);
#endif

 // BEM280 sensor object with I2C protokol
#if WITH_BME280 > 0
  Adafruit_BME280 bme;
#endif

volatile long w132fifoBuf[FIFOSIZE]; // ring buffer
volatile byte w132fifoReadIndex, w132fifoWriteIndex;  // ring buffer read and write index W132

volatile long w174fifoBuf[FIFOSIZE]; // ring buffer
volatile byte w174fifoReadIndex, w174fifoWriteIndex;  // ring buffer read and write index W174

byte ventusReceivedBits = 0; // if set bit 0 = temperature & humidity, bit 1 = wind speed, bit 2 = wind direction & gust, bit 3 = rain

void setup()
{
  debugStart(SERIALSPEED);
  debugln(F("Start receiving and decoding Ventus Weather Sensors via cable."));

  #if WITH_MQTT > 0
    client.setServer(MQTT_SERVER, MQTT_SERVERPORT);
    client.setCallback(mqttCallback);
    wifiConnect();
  #endif

  // WITH_BME280 sensor setup
  #if WITH_BME280 > 0
    Wire.begin(0, 2); // D3/GPIO0 connected to SDA, D4/GPIO2 connected to SCL on WITH_BME280 sensor
    Wire.setClock(100000);
    if (!bme.begin(0x76))
    {
      debugln("Could not find a valid WITH_BME280 sensor, check wiring!");
      while (1);
    }

    //forced mode, 1x temperature / 1x humidity / 1x pressure oversampling, filter off
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
  #endif

  // W132 and W174 weather sensors setup
  pinMode(W132_DATAPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(W132_DATAPIN), w132Handler, CHANGE);
  pinMode(W174_DATAPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(W174_DATAPIN), w174Handler, CHANGE);
  ventusReceivedBits = 0;
}

boolean w132CRCValid(unsigned long value, byte checksum)
// check if w132 received crc is correct for received value
{
  byte calculatedChecksum = 0xF;
  for (int i = 0 ; i < 8 ; i++) calculatedChecksum -= (byte)(value >> (i * 4));
  calculatedChecksum &= 0xF;
  return (calculatedChecksum == checksum);
}

boolean w174CRCValid(unsigned long value, byte checksum)
// check if w174 received crc is correct for received value
{
  byte calculatedChecksum = 0x7;
  for (int i = 0 ; i < 8 ; i++) calculatedChecksum += (byte)(value >> (i * 4));
  calculatedChecksum &= 0xF;
  return (calculatedChecksum == checksum);
}

void w132Handler()
{
  static long w132LineUp, w132LineDown;
  static unsigned long w132DataBits = 0;
  static byte w132CRCBits = 0;
  static byte w132BitsCounted = 0;
  long w132ZeroTime, w132OneTime;

  byte w132State = digitalRead(W132_DATAPIN); // current pin state

  if (w132State) // pin is now HIGH
  {
    w132LineUp = micros(); // line went HIGH after being LOW at this time
    w132ZeroTime = w132LineUp - w132LineDown; // calculate the LOW pulse time
    if (w132ZeroTime > VENTUS_SYNC - VENTUS_TOL && w132ZeroTime < VENTUS_SYNC + VENTUS_TOL)
    {
      // new start message
      w132DataBits = 0;
      w132CRCBits = 0;
      w132BitsCounted = 0;
    }
    else if (w132ZeroTime > VENTUS_ONE - VENTUS_TOL && w132ZeroTime < VENTUS_ONE + VENTUS_TOL)
    { // set the one bits
      if (w132BitsCounted < VENTUS_DATALEN)
        bitSet(w132DataBits, w132BitsCounted);
      else
        bitSet(w132CRCBits, w132BitsCounted - VENTUS_DATALEN);
      w132BitsCounted++;
    }
    else if (w132ZeroTime > VENTUS_ZERO - VENTUS_TOL && w132ZeroTime < VENTUS_ZERO + VENTUS_TOL)
    { // setting zero bits is not necessary, but count them
      w132BitsCounted++;
    }
    else // received bit is not a SYNC, ONE or ZERO bit, so restart
    {
      w132DataBits = 0;
      w132CRCBits = 0;
      w132BitsCounted = 0;
    }

    if (w132BitsCounted >= VENTUS_DATALEN + VENTUS_CRCLEN) // all bits received
    {
      // write valid value to buffer
      if (w132CRCValid(w132DataBits, w132CRCBits))
      {
        w132fifoWrite(w132DataBits);
      }
      else
      {
        w132fifoWrite(0); // write 0 to FIFO buffer (0 = invalid value received)
      }
      w132DataBits = 0;
      w132CRCBits = 0;
      w132BitsCounted = 0;
    }
  }
  else
  { // High values have no information with them
    w132LineDown = micros(); // line went LOW after being HIGH
    w132OneTime = w132LineDown - w132LineUp; // calculate the HIGH pulse time
  }
}

void w174Handler()
{
  static long w174LineUp, w174LineDown;
  static unsigned long w174DataBits = 0;
  static byte w174CRCBits = 0;
  static byte w174BitsCounted = 0;
  long w174ZeroTime, w174OneTime;

  byte w174State = digitalRead(W174_DATAPIN); // current pin state

  if (w174State) // pin is now HIGH
  {
    w174LineUp = micros(); // line went HIGH after being LOW at this time
    w174ZeroTime = w174LineUp - w174LineDown; // calculate the LOW pulse time
    if (w174ZeroTime > VENTUS_SYNC - VENTUS_TOL && w174ZeroTime < VENTUS_SYNC + VENTUS_TOL)
    {
      // new start message
      w174DataBits = 0;
      w174CRCBits = 0;
      w174BitsCounted = 0;
    }
    else if (w174ZeroTime > VENTUS_ONE - VENTUS_TOL && w174ZeroTime < VENTUS_ONE + VENTUS_TOL)
    { // set the one bits
      if (w174BitsCounted < VENTUS_DATALEN)
        bitSet(w174DataBits, w174BitsCounted);
      else
        bitSet(w174CRCBits, w174BitsCounted - VENTUS_DATALEN);
      w174BitsCounted++;
    }
    else if (w174ZeroTime > VENTUS_ZERO - VENTUS_TOL && w174ZeroTime < VENTUS_ZERO + VENTUS_TOL)
    { // setting zero bits is not necessary, but count them
      w174BitsCounted++;
    }
    else // received bit is not a SYNC, ONE or ZERO bit, so restart
    {
      w174DataBits = 0;
      w174CRCBits = 0;
      w174BitsCounted = 0;
    }

    if (w174BitsCounted >= VENTUS_DATALEN + VENTUS_CRCLEN) // all bits received
    {
      // write valid value to buffer
      if (w174CRCValid(w174DataBits, w174CRCBits))
      {
        w174fifoWrite(w174DataBits);
      }
      else
      {
        w174fifoWrite(0); // write 0 to FIFO buffer (0 = invalid value received)
      }
      w174DataBits = 0;
      w174CRCBits = 0;
      w174BitsCounted = 0;
    }
  }
  else
  { // High values have no information with them
    w174LineDown = micros(); // line went LOW after being HIGH
    w174OneTime = w174LineDown - w174LineUp; // calculate the HIGH pulse time
  }
}

void w132fifoWrite(long item)
// write item into ring buffer
{
  w132fifoBuf[w132fifoWriteIndex] = item; // store the item
  if (!(w132fifoWriteIndex + 1 == w132fifoReadIndex || (w132fifoWriteIndex + 1 >= FIFOSIZE && w132fifoReadIndex == 0)))
    w132fifoWriteIndex++;  // advance write pointer in ringbuffer
  if (w132fifoWriteIndex >= FIFOSIZE) w132fifoWriteIndex = 0; // ring buffer is at its end
}

void w174fifoWrite(long item)
// write item into ring buffer
{
  w174fifoBuf[w132fifoWriteIndex] = item; // store the item
  if (!(w174fifoWriteIndex + 1 == w174fifoReadIndex || (w174fifoWriteIndex + 1 >= FIFOSIZE && w174fifoReadIndex == 0)))
    w174fifoWriteIndex++;  // advance write pointer in ringbuffer
  if (w174fifoWriteIndex >= FIFOSIZE) w174fifoWriteIndex = 0; // ring buffer is at its end
}

unsigned long w132fifoRead()
// always check first if item is available with fifoAvailable()
// before reading the ring buffer using this function
{
  unsigned long item;
  item = w132fifoBuf[w132fifoReadIndex];
  detachInterrupt(digitalPinToInterrupt(W132_DATAPIN)); // Interrupts off while changing the read pointer for the ringbuffer
  w132fifoBuf[w132fifoReadIndex] = 0;
  w132fifoReadIndex++;
  if (w132fifoReadIndex >= FIFOSIZE) w132fifoReadIndex = 0;
  attachInterrupt(digitalPinToInterrupt(W132_DATAPIN), w132Handler, CHANGE); // Interrupts on again
  return (item);
}

unsigned long w174fifoRead()
// always check first if item is available with fifoAvailable()
// before reading the ring buffer using this function
{
  unsigned long item;
  item = w174fifoBuf[w174fifoReadIndex];
  detachInterrupt(digitalPinToInterrupt(W174_DATAPIN)); // Interrupts off while changing the read pointer for the ringbuffer
  w174fifoBuf[w174fifoReadIndex] = 0;
  w174fifoReadIndex++;
  if (w174fifoReadIndex >= FIFOSIZE) w174fifoReadIndex = 0;
  attachInterrupt(digitalPinToInterrupt(W174_DATAPIN), w174Handler, CHANGE); // Interrupts on again
  return (item);
}

boolean w132fifoAvailable()
// item is available for reading if (fifoReadIndex!=fifoWriteIndex)
{
  return (w132fifoReadIndex != w132fifoWriteIndex);
}

boolean w174fifoAvailable()
// item is available for reading if (fifoReadIndex!=fifoWriteIndex)
{
  return (w174fifoReadIndex != w174fifoWriteIndex);
}

void w132PrintResults(unsigned long value)
{
  #if WITH_DEBUG > 0 && WITH_DEBUG_SENSORS > 0
  debugln("----------========== W132 ==========----------");

  debug("Telegram: ");
  for (byte i = 0; i < 32; i++)
  {
    debug(value >> i & 0b1);
  }
  debugln("");

  // Valid Values
  byte w132ResultCode = 0;

  // Battery State
  bool lowBattery = (value >> 8 & 0b1);    // bit 8 is set if battery voltage is low
  debug("Battery Low: ");
  debugln(lowBattery);

  // Trigger
  bool forcedSend = (value >> 11 & 0b1); // bit 11 is set if manual send button was pressed
  debug("Trigger: ");
  debugln(forcedSend);

  // Temperature Trend
  byte trend = (value >> 9 & 0b11); // bit 9, 10

  // if the Trend value is 3, then it's not a temperature message, but a wind message
  if (trend != 3)
  {
    // Temperature Trend
    char tempTrend;
    if (trend == 0) tempTrend = '=';     // temp tendency steady
    else if (trend == 1) tempTrend = '-'; // temp tendency falling
    else if (trend == 2) tempTrend = '+'; // temp tendency rising

    // Temperature (C)
    int temp = (value >> 12 & 0b11111111111); // bit 12..22
    if (temp > 600)
    {
      w132ResultCode = W132_ERROR_TEMPERATURE;
    }

    // bit 23 is sign
    byte negative = (value >> 23 & 0b1);
    if (negative == 1)
    {
      temp = -2048 + temp;
    }
    float temperature = (float)temp / 10;

    // Humidity (%)
    byte humidityOnes = value >> 24 & 0b1111; // bit 24..27
    byte humidityTens = value >> 28 & 0b1111; // bit 28..31
    byte humidity = (humidityTens * 10) + humidityOnes;

    bitSet(ventusReceivedBits, W132_RECEIVED_TEMPHUM);

    debug("Temperature: ");
    debugln(temperature);
    debug("        RAW: ");
    debugln(temp);

    debug("Humidity: ");
    debugln(humidity);
    debug("Ones RAW: ");
    debugln(humidityOnes);
    debug("Tens RAW: ");
    debugln(humidityTens);
  }
  else
  {
    byte windType = (value >> 12 & 0b111); // bit 12..14, 111 = Wind Direction & Gust, otherwise Wind Speed

    if (windType == 7)
    {
      // Wind Direction (grad)
      unsigned int windDirection = (value >> 15 & 0b111111111); // bit 15..23
      if (windDirection < 0 || windDirection > 360)
      {
        w132ResultCode = W132_ERROR_WINDDIRECTION;
      }

      // Wind Gust (m/s), bit 24..31
      float windGust = (float)(value >> 24 & 0b11111111) / 5;

      bitSet(ventusReceivedBits, W132_RECEIVED_WINDDIRGUST);

      debug("Wind Direction: ");
      debugln(windDirection);
      debug("           RAW: ");
      debugln(value >> 15 & 0b111111111);

      debug("Wind Gust: ");
      debugln(windGust);
      debug("      RAW: ");
      debugln(value >> 24 & 0b11111111);
    }
    else
    {
      // Wind Speed (m/s), bit 24..31
      float windSpeed = (float)(value >> 24 & 0b11111111) / 5;

      bitSet(ventusReceivedBits, W132_RECEIVED_WINDSPEED);

      debug("Wind Speed: ");
      debugln(windSpeed);
      debug("       RAW: ");
      debugln(value >> 24 & 0b11111111);
    }
  }

  debugln("");
  #endif
}

void w132DecodeResults(unsigned long value)
{
  // Valid Values
  byte w132ResultCode = 0;

  // Battery State
  bool lowBattery = (value >> 8 & 0b1);    // bit 8 is set if battery voltage is low

  // Trigger
  bool forcedSend = (value >> 11 & 0b1); // bit 11 is set if manual send button was pressed

  // Temperature Trend
  byte trend = (value >> 9 & 0b11); // bit 9, 10

  // if the Trend value is 3, then it's not a temperature message, but a wind message
  if (trend != 3)
  {
    // Temperature Trend
    char tempTrend;
    if (trend == 0) tempTrend = '=';     // temp tendency steady
    else if (trend == 1) tempTrend = '-'; // temp tendency falling
    else if (trend == 2) tempTrend = '+'; // temp tendency rising

    // Temperature (C)
    int temp = (value >> 12 & 0b11111111111); // bit 12..22
    if (temp > 600)
    {
      w132ResultCode = W132_ERROR_TEMPERATURE;
    }

    // bit 23 is sign
    byte negative = (value >> 23 & 0b1);
    if (negative == 1)
    {
      temp = -2048 + temp;
    }
    float temperature = (float)temp / 10;

    // Humidity (%)
    byte humidityOnes = value >> 24 & 0b1111; // bit 24..27
    byte humidityTens = value >> 28 & 0b1111; // bit 28..31
    byte humidity = (humidityTens * 10) + humidityOnes;

    bitSet(ventusReceivedBits, W132_RECEIVED_TEMPHUM);
  }
  else
  {
    byte windType = (value >> 12 & 0b111); // bit 12..14, 111 = Wind Direction & Gust, otherwise Wind Speed

    if (windType == 7)
    {
      // Wind Direction (grad)
      unsigned int windDirection = (value >> 15 & 0b111111111); // bit 15..23
      if (windDirection < 0 || windDirection > 360)
      {
        w132ResultCode = W132_ERROR_WINDDIRECTION;
      }

      // Wind Gust (m/s), bit 24..31
      float windGust = (float)(value >> 24 & 0b11111111) / 5;

      bitSet(ventusReceivedBits, W132_RECEIVED_WINDDIRGUST);
    }
    else
    {
      // Wind Speed (m/s), bit 24..31
      float windSpeed = (float)(value >> 24 & 0b11111111) / 5;

      bitSet(ventusReceivedBits, W132_RECEIVED_WINDSPEED);
    }
  }
}

void w132PublishResults(unsigned long value)
{
  #if WITH_MQTT > 0

  const int capacity = JSON_OBJECT_SIZE(100);
  StaticJsonBuffer<capacity> jsonBuffer;
    
  // Create JsonObject
  JsonObject &root = jsonBuffer.createObject();

  // Add identification
  JsonObject &id = root.createNestedObject("identification");
  id["type"] = "sensor";
  id["id"] = 1;
  id["name"] = "W132";
  id["position"] = "Arbeitszimmer";

  // Add sensor datas
  JsonObject &data = root.createNestedObject("data");
  data["trigger"] = (value >> 11 & 0b1);
  data["battery_low"] = (value >> 8 & 0b1);
  data["temperature"] = "n/a";
  data["temperature_trend"] = "n/a";
  data["humidity"] = "n/a";
  data["wind_speed"] = "n/a";
  data["wind_direction"] = "n/a";
  data["wind_gust"] = "n/a";

  // Temperature Trend
  byte trend = (value >> 9 & 0b11); // bit 9, 10

  // if the Trend value is 3, then it's not a temperature message, but a wind message
  if (trend != 3)
  {
    // Temperature Trend
    data["temperature_trend"] = trend;
    
    // Temperature (C)
    int temp = (value >> 12 & 0b11111111111); // bit 12..22

    // sign bit 23
    if ((value >> 23 & 0b1) == 1)
    {
      temp = -2048 + temp;
    }
    data["temperature"] = temp;

    // Humidity (%)
    byte humidityOnes = (value >> 24 & 0b1111); // bit 24..27
    byte humidityTens = (value >> 28 & 0b1111); // bit 28..31
    data["humidity"] = (humidityTens * 10) + humidityOnes;

    bitSet(ventusReceivedBits, W132_RECEIVED_TEMPHUM);
  }
  else
  {
    byte windType = (value >> 12 & 0b111); // bit 12..14, 111 = Wind Direction & Gust, otherwise Wind Speed

    if (windType == 7)
    {
      // Wind Direction (grad)
      data["wind_direction"] = (value >> 15 & 0b111111111); // bit 15..23

      // Wind Gust (m/s), bit 24..31
      data["wind_gust"] = (value >> 24 & 0b11111111);

      bitSet(ventusReceivedBits, W132_RECEIVED_WINDDIRGUST);
    }
    else
    {
      // Wind Speed (m/s), bit 24..31
      data["wind_speed"] = (value >> 24 & 0b11111111);
      
      bitSet(ventusReceivedBits, W132_RECEIVED_WINDSPEED);
    }
  }

  char publishJson[256];
  root.printTo(publishJson);
  debug("MQTT topic: ");
  debugln(sensorTopic);
  debug("MQTT publish: ");
  debugln(publishJson);
  if(client.publish(sensorTopic, (const char*)publishJson), true)
  //if(client.publish(sensorTopic, "Sensor W132"))
  {
    debugln("MQTT published.");
  }
  else
  {
    debugln("MQTT publishing failed!");
  }
  #endif
}

void w174PrintResults(unsigned long value)
{
  #if WITH_DEBUG > 0 && WITH_DEBUG_SENSORS > 0
  
  // Valid Values
  byte w174ResultCode = 0;

  // Battery State
  bool lowBattery = (value >> 8 & 0b1);    // bit 8 is set if battery voltage is low

  // Rain (mm)
  float rain = (value >> 16 & 0b1111111111111111) * 0.25; // bits 16..31

  bitSet(ventusReceivedBits, W174_RECEIVED_RAIN);

  debugln("----------========== W174 ==========----------");
  debug("Telegram: ");
  for (byte i = 0; i < 32; i++)
  {
    debug(value >> i & 0b1);
  }
  debugln("");

  debug("Battery Low: ");
  debugln(lowBattery);
  debug("Rain: ");
  debugln(rain);
  debug(" RAW: ");
  debugln((float)(value >> 16 & 0b1111111111111111));

  #endif
}

void w174DecodeResults(unsigned long value)
{
  // Valid Values
  byte w174ResultCode = 0;

  // Battery State
  bool lowBattery = (value >> 8 & 0b1);    // bit 8 is set if battery voltage is low

  // Rain (mm)
  float rain = (value >> 16 & 0b1111111111111111) * 0.25; // bits 16..31

  bitSet(ventusReceivedBits, W174_RECEIVED_RAIN);
}

void w174PublishResults(unsigned long value)
{ 
  #if WITH_MQTT > 0

  const int capacity = JSON_OBJECT_SIZE(100);
  StaticJsonBuffer<capacity> jsonBuffer;
  
  // Create JsonObject
  JsonObject &root = jsonBuffer.createObject();

  // Add identification
  JsonObject &id = root.createNestedObject("identification");
  id["type"] = "sensor";
  id["id"] = 2;
  id["name"] = "W174";
  id["position"] = "Arbeitszimmer";

  // Add sensor datas
  JsonObject &data = root.createNestedObject("data");
  data["battery_low"] = (value >> 8 & 0b1);
  data["rain"] = (value >> 16 & 0b1111111111111111); // bits 16..31

  bitSet(ventusReceivedBits, W174_RECEIVED_RAIN);

  //root.printTo(Serial);
  char publishJson[256];
  root.printTo(publishJson);
  debug("MQTT topic: ");
  debugln(sensorTopic);
  debug("MQTT publish: ");
  debugln(publishJson);
  if(client.publish(sensorTopic, (const char*)publishJson), true)
  //if(client.publish(sensorTopic, "Sensor W174"))
  {
    debugln("MQTT published.");
  }
  else
  {
    debugln("MQTT publishing failed!");
  }

  #endif
}

void bme280PrintResults()
{
  #if WITH_BME280 > 0 && WITH_DEBUG > 0 && WITH_DEBUG_SENSORS > 0
  
  debugln("----------========= WITH_BME280 =========----------");
  debug("Temperature: ");
  debugln(bme.readTemperature());
  debug("Humidity: ");
  debugln(bme.readHumidity());
  debug("Pressure: ");
  debugln(bme.readPressure() / 100.0F);
  debug("     RAW: ");
  debugln(bme.readPressure());
  debug("Approx. Altitude: ");
  debugln(bme.readAltitude(SEALEVELPRESSURE_HPA));
  debugln("");

  #endif
}

void bme280PublishResults()
{
  #if WITH_MQTT > 0 &&  WITH_BME280 > 0
  
  // Create JsonBuffer
  const int capacity = JSON_OBJECT_SIZE(100);
  StaticJsonBuffer<capacity> jsonBuffer;
  
  // Create JsonObject
  JsonObject &root = jsonBuffer.createObject();

  // Add identification
  JsonObject &id = root.createNestedObject("identification");
  id["type"] = "sensor";
  id["id"] = 3;
  id["name"] = "BME280";
  id["position"] = "Arbeitszimmer";

  // Add sensor datas
  JsonObject &data = root.createNestedObject("data");
  data["temperature"] = bme.readTemperature();
  data["humidity"] = bme.readHumidity();
  data["pressure"] = bme.readPressure();
  data["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);

  char publishJson[256];
  root.printTo(publishJson);
  debug("MQTT topic: ");
  debugln(sensorTopic);
  debug("MQTT publish: ");
  debugln(publishJson);
  if(client.publish(sensorTopic, (const char*)publishJson), true)
  //if(client.publish(sensorTopic, "Sensor BME"))
  {
    debugln("MQTT published.");
  }
  else
  {
    debugln("MQTT publishing failed!");
  }

  #endif
}

// Connect to WiFi access point.
void wifiConnect()
{
  #if WITH_MQTT > 0
  
  debug("Connecting to WiFi ");
  debug(WLAN_SSID);
    
  WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    debug(".");
  }
  debugln();
  debug("WiFi connected. ");
  debug("IP address: ");
  debugln(WiFi.localIP());

  #endif  
}

// Function mqttCallback
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  #if WITH_MQTT > 0 && WITH_DEBUG > 0 && WITH_MQTT_DEBUG > 0
  
  debug("Message arrived [");
  debug(topic);
  debugln("] ");

  for (int i = 0; i < length; i++)
  {
    debug((char)payload[i]);
  }
  debugln();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    debugln("success");
  } 
  else
  {
    debugln("failed");
  }

  #endif
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
boolean mqttConnect()
{
  #if WITH_MQTT > 0

  // Loop until we're reconnected
  while (!client.connected()) {
    if (WITH_DEBUG_MQTT > 0) debugln("Attempting MQTT connection...");
    
    // Create a random client ID
    String clientId = "ESP8266Client-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str()))
    {
      if (WITH_DEBUG_MQTT > 0) debugln("Broker Server connected");

      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");

      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      if (WITH_DEBUG_MQTT > 0)
      {
        debug("Broker Server connection failed, rc=");
        debugln(client.state());
        debugln(" try again in 5 seconds");
      }
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  
  client.loop();
  
  #endif
}

void loop()
{
  if (w132fifoAvailable())
  {
    unsigned long dataReceived = w132fifoRead();
    if (dataReceived > 0)
    {
      #if WITH_DEBUG > 0 && WITH_DEBUG_SENSORS > 0
        w132PrintResults(dataReceived);
      #endif  

      #if WITH_MQTT > 0
        mqttConnect();
        w132PublishResults(dataReceived);  
      #endif

      #if WITH_MQTT == 0 && (WITH_DEBUG == 0 || WITH_DEBUG_SENSORS == 0)
        w132DecodeResults(dataReceived);  
      #endif

      #if WITH_DEBUG > 0 && WITH_DEBUG_SENSORS > 0
        debug("Received Bits: ");
        for (byte i = 0; i < 4; i++)
        {
          debug(ventusReceivedBits >> i & 0b1);
        }
        debugln();
      #endif

      if (ventusReceivedBits == 1 || ventusReceivedBits >= 6)
      {
        ventusReceivedBits = 0;
        detachInterrupt(digitalPinToInterrupt(W132_DATAPIN)); // Interrupts off while sleeping
        detachInterrupt(digitalPinToInterrupt(W174_DATAPIN)); // Interrupts off while sleeping

        #if WITH_BME280 > 0
          #if WITH_DEBUG > 0 && WITH_DEBUG_SENSORS > 0
            bme280PrintResults();
          #endif
            
          #if WITH_MQTT > 0
            mqttConnect();
            bme280PublishResults();
          #endif
        #endif 

        #if WITH_DEEPSLEEP > 0
          // disconnect WiFi
          debugln("disconnecting WiFi.");
          WiFi.disconnect();

          // disconnect MQTT
          debugln("disconnecting MQTT.");
          client.disconnect();
          
          debug("deep sleeping ");
          debug(SLEEP_S);
          debugln(" seconds ...");
          
          ESP.deepSleep(SLEEP_S * 1000000, WAKE_RF_DEFAULT); // xx secs.
          delay(100);
        #else
          debug("delay ");
          debug(SLEEP_S);
          debugln(" seconds ...");
          delay(SLEEP_S * 1000);
          debugln("continue");
          mqttConnect();
          client.loop();

          attachInterrupt(digitalPinToInterrupt(W132_DATAPIN), w132Handler, CHANGE);
          attachInterrupt(digitalPinToInterrupt(W174_DATAPIN), w174Handler, CHANGE);
        #endif
      }
    }
  }

  // W174 is treated a bit carelessly, the values come with enough frequency in the period of W132
  if (w174fifoAvailable())
  {
    unsigned long dataReceived = w174fifoRead();
    if (dataReceived > 0)
    {
      #if WITH_DEBUG > 0
        w174PrintResults(dataReceived);
        #if WITH_MQTT > 0
          mqttConnect();
          w174PublishResults(dataReceived);  
        #endif
      #elif WITH_MQTT > 0
        w174PublishResults(dataReceived);
      #else
        w174DecodeResults(dataReceived);  
      #endif

      #if WITH_DEBUG > 0
        debug("Received Bits: ");
        for (byte i = 0; i < 4; i++)
        {
          debug(ventusReceivedBits >> i & 0b1);
        }
        debugln();
      #endif
    }
  }
}
