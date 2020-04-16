
#define DEBUG
#define DEBUGDELAY 10000

#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi101.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>

#ifndef DEBUG
#include <RTCZero.h>
#include <Adafruit_SleepyDog.h>
#endif

#define CMD_INTERVAL 500

#define DEVICE_NAME "OutsideThermometer"

//#define VOLTAGE

#define TEMPERATURE_TOPIC   "living_room/sensors/temperatureout"
#define HUMIDITY_TOPIC      "living_room/sensors/humidityout"
#define VOLTAGE_TOPIC       "living_room/sensors/voltageout"

Adafruit_BME280 bme; // I2C

#ifndef DEBUG
RTCZero rtc;
#endif

// Set some dummy data, since we just want intervals.
const uint8 seconds = 0;
const uint8 minutes = 0;
const uint8 hours   = 0;
const uint8 day     = 1;
const uint8 month   = 1;
const uint8 year    = 17;

#define ANALOG_BITS 12
#define BITS        4096
#define VREF        3.3
#define R1          456
#define R2          978

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long delayTime;

void getNextSample(float *Temperature,
                   float *Humidity)
{
  bme.takeForcedMeasurement();

  *Humidity    = bme.readHumidity();
  *Temperature = bme.readTemperature();
}

void connectWifi()
{
  Serial.print("WiFi connecting ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(CMD_INTERVAL);
  }

  Serial.print(" connected\r\n");
}

void disconnectWifi()
{
  Serial.print("WiFi disconnecting ");

  WiFi.disconnect();
  while (WiFi.status() == WL_CONNECTED)
  {
    delay(CMD_INTERVAL);
    Serial.print(".");
  }
  Serial.print(" disconnected\r\n");
}

void connectMQTT()
{
  Serial.print("MQTT connecting ");
  while (!mqtt.connected())
  {
    if (mqtt.connect(DEVICE_NAME))
    {
      Serial.print(" connected");
    }
    else
    {
      delay(CMD_INTERVAL);
      Serial.print(".");
    }
  }
  Serial.println("");
}

void disconnectMQTT()
{
  Serial.print("MQTT disconnecting ");

  mqtt.disconnect();
  while(mqtt.connected())
  {
    delay(CMD_INTERVAL);
    Serial.print(".");
  }
  Serial.print(" disconnected\r\n");
}

void setup()
{

  Serial.begin(9600);

  analogReadResolution(ANALOG_BITS);

  mqtt.setServer(MQTT_SERVER, MQTT_SERVERPORT);

  pinMode(6UL, OUTPUT);
  WiFi.hostname("OutsideThermometer");
  bme.begin(0x76, &Wire);

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_NONE, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

#ifndef DEBUG
  // Set the RTC
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);
  rtc.enableAlarm(rtc.MATCH_MMSS);
#endif
}

void work()
{
  float temperature, humidity;

  digitalWrite(6UL, HIGH);
  getNextSample(&temperature, &humidity);

  Serial.print("Humidity:      ");
  Serial.print(humidity);
  Serial.print(" %\r\n");

  Serial.print("Temperature:   ");
  Serial.print(temperature);
  Serial.print(" °C\r\n");

#ifdef VOLTAGE
  uint16 Vin = analogRead(A1);
  float Vout = ((double)Vin * VREF / BITS) * (R1 + R2) / R2;
  Serial.print("VBAT:     ");
  Serial.print(Vout);
  Serial.print(" V\r\n");
#endif

#ifndef DEBUG
  Watchdog.reset();
#endif

  connectWifi();
  connectMQTT();

#ifndef DEBUG
  Watchdog.reset();
#endif

  Serial.print("Sending ... ");
  mqtt.publish(TEMPERATURE_TOPIC, String(temperature).c_str(),  true);
  mqtt.publish(HUMIDITY_TOPIC,    String(humidity).c_str(),     true);
#ifdef VOLTAGE
  mqtt.publish(VOLTAGE_TOPIC,     String(Vout).c_str(),         true);
#endif
  Serial.print("done\r\n");

#ifndef DEBUG
  Watchdog.reset();
#endif

  disconnectMQTT();
  disconnectWifi();
  digitalWrite(6, LOW);
}

void loop() {

  Serial.begin(9600);

  Serial.println("-------------------------");

#ifndef DEBUG
  Watchdog.enable(8000);
#endif

  work(); // do the work

#ifdef DEBUG
  delay(DEBUGDELAY);
#else
  rtc.setAlarmMinutes((rtc.getAlarmMinutes() + SENDING_INTERVAL) % 60);
  Watchdog.disable();
  rtc.standbyMode();
#endif
}
