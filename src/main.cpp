/*
 * MKR1000-DHT22-MQTT.ino
 */
#include "config.h"
#include <Arduino.h>
#include <WiFi101.h>
#include <WiFiClient.h>
#include <RTCZero.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <Adafruit_SleepyDog.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define CMD_INTERVAL 500

#define DHTPIN 2	  // what digital pin we're connected to
#define DHTTYPE DHT22 // DHT 11 is also possible

#define TEMPERATURE_TOPIC "kitchen/sensors/temperatureout"
#define HUMIDITY_TOPIC "kitchen/sensors/humidityout"
#define VOLTAGE_TOPIC "kitchen/sensors/voltageout"

DHT dht(DHTPIN, DHTTYPE);
RTCZero rtc;

// Set some dummy data, since we just want intervals.
const uint8 seconds = 0;
const uint8 minutes = 0;
const uint8 hours = 0;
const uint8 day = 1;
const uint8 month = 1;
const uint8 year = 17;

#define ANALOG_BITS 12
#define BITS 4096
#define VREF 3.3
#define R1 456
#define R2 978

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

void getNextSample(float *Temperature,
				   float *Humidity)
{
	*Humidity = dht.readHumidity();
	*Temperature = dht.readTemperature();
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

	Serial.print(" done\r\n");
}

void disconnectWifi()
{
	WiFi.disconnect();
	Serial.print("WiFi disconnecting ");
	while (WiFi.status() == WL_CONNECTED)
	{
		delay(CMD_INTERVAL);
		Serial.print(".");
	}
	Serial.print("done\r\n");
}

void connectMQTT()
{
	Serial.print("MQTT connecting ");
	while (!mqtt.connected())
	{
		Serial.print("Attempting MQTT connection...");
		// Attempt to connect
		if (mqtt.connect("OutsideThermometer"))
		{
			Serial.println("connected");
		}
		else
		{
			Serial.print("failed, rc=");
			Serial.print(mqtt.state());
			Serial.println(" try again in 5 seconds");
			// Wait 5 seconds before retrying
			delay(5000);
		}
	}

	Serial.print("done\r\n");
}

void disconnectMQTT()
{
	mqtt.disconnect();
	Serial.print("MQTT disconnect\r\n");
}

void setup()
{
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

void work()
{
	float temperature, humidity;

	digitalWrite(6UL, HIGH);
	getNextSample(&temperature, &humidity);

	Serial.print("Humidity: ");
	Serial.print(humidity);
	Serial.print("\r\n");

	Serial.print("Temperature: ");
	Serial.print(temperature);
	Serial.print("\r\n");
	Watchdog.reset();

	uint16 Vin = analogRead(A1);
	float Vout = ((double)Vin * VREF / BITS) * (R1 + R2) / R2;
	Serial.print("VBAT: ");
	Serial.print(Vout);
	Serial.print("\r\n");

	connectWifi();
	connectMQTT();

	Watchdog.reset();
	mqtt.publish(TEMPERATURE_TOPIC, String(temperature).c_str(), true);
	mqtt.publish(HUMIDITY_TOPIC, String(humidity).c_str(), true);
	mqtt.publish(VOLTAGE_TOPIC, String(Vout).c_str(), true);

	Watchdog.reset();

	disconnectMQTT();
	disconnectWifi();
	digitalWrite(6, LOW);
}

void loop()
{
	Watchdog.enable(8000);

	work(); // do the work
	rtc.setAlarmMinutes((rtc.getAlarmMinutes() + SENDING_INTERVAL) % 60);

	Watchdog.disable();
	rtc.standbyMode();
}
