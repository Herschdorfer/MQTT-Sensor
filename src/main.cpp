/* CONFIG */
/* Debug config */
//#define DEBUG // to enable debug
#define DEBUGDELAY 10000 // how often to send in debug mode

/* Invall to try to disconnect/connect to WiFi/MQTT */
#define CMD_INTERVAL 500

/* The device name */
#define DEVICE_NAME "OutsideThermometer"

/* Battery voltage settings */
/* uncomment if you use a voltage measurement on the battery */
#define VOLTAGE
#define ANALOG_BITS 12
#define BITS        4096
#define V_REF       3.3
#define R1          456
#define R2          978
#define V_BAT_IN    A1

/* MQTT topics to be used */
#define TEMPERATURE_TOPIC   "outside/sensors/temperature"
#define HUMIDITY_TOPIC      "outside/sensors/humidity"
#define VOLTAGE_TOPIC       "outside/sensors/voltage"

/* includes */
#include "config.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>

/* WIFI include */
#ifdef ARDUINO_SAMD_MKR1000
#include <WiFi101.h>
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#include <WiFiNINA.h>
#else
#error "target unknown"
#endif

/* includes not used in debug mode */
#ifndef DEBUG
#include <RTCZero.h>
#include <Adafruit_SleepyDog.h>
#endif

static Adafruit_BME280 bme; // I2C

#ifndef DEBUG
static RTCZero rtc;
#endif

// Set some dummy data, since we just want intervals.
const uint8_t seconds = 0;
const uint8_t minutes = 0;
const uint8_t hours   = 0;
const uint8_t day     = 1;
const uint8_t month   = 1;
const uint8_t year    = 17;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

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
#ifdef ARDUINO_SAMD_MKR1000
  WiFi.hostname(DEVICE_NAME);
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
  WiFi.setHostname(DEVICE_NAME);
#else
#error "target unknown"
#endif
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
  uint16_t Vin = analogRead(V_BAT_IN);
  float Vout = ((double)Vin * V_REF / BITS) * (R1 + R2) / R2;
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
