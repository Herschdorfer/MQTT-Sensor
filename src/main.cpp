/*
 * MKR1000-DHT22-MQTT.ino
 */
#include "config.h"
#include <Arduino.h>
#include <WiFi101.h>
#include <WiFiClient.h>
#include <RTCZero.h>
#include <DHT.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
//#include <Adafruit_SleepyDog.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define CMD_INTERVAL 500

#define DHTPIN 2        // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 11 is also possible

#define TEMPERATURE_TOPIC "kitchen/sensors/temperatureout"
#define HUMIDITY_TOPIC "kitchen/sensors/humidityout"

double temperature, humidity;

DHT dht(DHTPIN, DHTTYPE);
RTCZero rtc;

// Set some dummy data, since we just want intervals.
const uint8 seconds = 0;
const uint8 minutes = 0;
const uint8 hours = 0;
const uint8 day = 1;
const uint8 month = 1;
const uint8 year = 17;

WiFiClient wifiClient;
Adafruit_MQTT_Client mqtt(&wifiClient, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_TOPIC, MQTT_QOS_0);
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_TOPIC, MQTT_QOS_0);

void getNextSample( double* Temperature,
					double* Humidity ) {
	*Humidity = dht.readHumidity();
	*Temperature = dht.readTemperature();
}

void connectWifi() {
	Serial.print("WiFi connecting ");
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(CMD_INTERVAL);
	}

	Serial.print(" done\r\n");
}

void disconnectWifi() {
	WiFi.disconnect();
	Serial.print("WiFi disconnecting ");
	while (WiFi.status() == WL_CONNECTED) {
		delay(CMD_INTERVAL);
		Serial.print(".");
	}
	Serial.print("done\r\n");
}

void connectMQTT() {
	Serial.print("MQTT connecting ");
	if (mqtt.connected()) {
		Serial.print("done\r\n");
		return;
	}

	while (mqtt.connect() != 0) {
		delay(CMD_INTERVAL);
		Serial.print(".");
	}

	Serial.print("done\r\n");
}

void disconnectMQTT() {
	mqtt.disconnect();
	Serial.print("MQTT disconnect\r\n");
}

void setup() {
	Serial.begin(57600);

	pinMode(6UL, OUTPUT);
	WiFi.hostname("OutsideThermometer");
	dht.begin();

	// Set the RTC
	rtc.begin();
	rtc.setTime(hours, minutes, seconds);
	rtc.setDate(day, month, year);
	rtc.setAlarmMinutes((rtc.getAlarmMinutes() + 1) % 60);
	rtc.enableAlarm(rtc.MATCH_MMSS);

	//rtc.standbyMode();
}

void messageReceived( 	String topic,
						String payload,
						char * bytes,
						unsigned int length ) {
}

void work() {
	digitalWrite(6UL, HIGH);
	getNextSample(&temperature, &humidity);

	Serial.print("Humidity: ");
	Serial.print(humidity);
	Serial.print("\r\n");

	Serial.print("Temperature: ");
	Serial.print(temperature);
	Serial.print("\r\n");
	//Watchdog.reset();

	connectWifi();
	connectMQTT();

	//Watchdog.reset();

    temperatureFeed.publish(temperature, 1);
    humidityFeed.publish(humidity, 1);

//	Watchdog.reset();

	disconnectMQTT();
	disconnectWifi();
	digitalWrite(6, LOW);

	delay(5000); // wait for the wifi to send the data
}

void loop() {
//	digitalWrite(6UL, HIGH);
	//Watchdog.enable(8000);

	work(); // do the work
	//rtc.setAlarmMinutes((rtc.getAlarmMinutes() + 15) % 60);

	//Watchdog.disable();
	//rtc.standbyMode();
}
