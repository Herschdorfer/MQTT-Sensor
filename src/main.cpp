/*
 * MKR1000-DHT22-MQTT.ino
 */
#include <Adafruit_Sensor.h>
#include <Adafruit_SleepyDog.h>
#include <Arduino.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <RTCZero.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiClient.h>

#include "config.h"

DHT dht(DHT_PIN, DHT_TYPE);
RTCZero rtc;

// Set some dummy data, since we just want intervals.
const uint8 seconds = 0;
const uint8 minutes = 0;
const uint8 hours = 0;
const uint8 day = 1;
const uint8 month = 1;
const uint8 year = 17;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

/**
 * @brief Get the Next Samples.
 *
 * @param[out] temperature  The temperature.
 * @param[out] humidity     Im a comment.
 */
void getNextSample(float *temperature, float *humidity) {
  *humidity = dht.readHumidity();
  *temperature = dht.readTemperature();
}

/**
 * @brief connects to Wifi.
 */
void connectWifi() {
  Serial.print("WiFi connecting ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(CMD_INTERVAL);
  }

  Serial.print(" done\r\n");
}

/**
 * @brief disconnects Wifi.
 */
void disconnectWifi() {
  WiFi.disconnect();
  Serial.print("WiFi disconnecting ");
  while (WiFi.status() == WL_CONNECTED) {
    delay(CMD_INTERVAL);
    Serial.print(".");
  }
  Serial.print("done\r\n");
}

/**
 * @brief Connects MQTT
 */
void connectMQTT() {
  Serial.print("MQTT connecting ");
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect("OutsideThermometer")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

  Serial.print("done\r\n");
}

/**
 * @brief disconnects MQTT
 *
 */
void disconnectMQTT() {
  mqtt.disconnect();
  Serial.print("MQTT disconnect\r\n");
}

void setup() {
  Serial.begin(57600);

  analogReadResolution(ANALOG_BITS);

  mqtt.setServer(MQTT_SERVER, MQTT_SERVERPORT);

  pinMode(6UL, OUTPUT);
  WiFi.hostname("OutsideThermometer");
  dht.begin();

  // Set the RTC
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);
  rtc.enableAlarm(rtc.MATCH_MMSS);
}

void work() {
  Serial.begin(57600);  // setup the UART again, if we start from sleep.

  float temperature = 0;
  float humidity = 0;

  digitalWrite(6UL, HIGH);
  getNextSample(&temperature, &humidity);

  Serial.print("humidity: ");
  Serial.print(humidity);
  Serial.print("\r\n");

  Serial.print("temperature: ");
  Serial.print(temperature);
  Serial.print("\r\n");
  Watchdog.reset();

  uint16 vIn = analogRead(A1);
  float vOut = ((double)vIn * V_REF / BITS) * (R1 + R2) / R2;
  Serial.print("V_BAT: ");
  Serial.print(vOut);
  Serial.print("\r\n");

  connectWifi();
  connectMQTT();

  Watchdog.reset();
  mqtt.publish(TEMPERATURE_TOPIC, String(temperature).c_str(), true);
  mqtt.publish(HUMIDITY_TOPIC, String(humidity).c_str(), true);
  mqtt.publish(VOLTAGE_TOPIC, String(vOut).c_str(), true);

  Watchdog.reset();

  disconnectMQTT();
  disconnectWifi();
  digitalWrite(6, LOW);
}

void loop() {
  Watchdog.enable(8000);

  work();  // do the work
  rtc.setAlarmMinutes((rtc.getAlarmMinutes() + SENDING_INTERVAL) % 60);

  Watchdog.disable();
  rtc.standbyMode();
}
