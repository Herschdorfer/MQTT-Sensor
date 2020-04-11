

#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi101.h>
#include <WiFiClient.h>
#include <RTCZero.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SleepyDog.h>

#define CMD_INTERVAL 500

#define TEMPERATURE_TOPIC 	"kitchen/sensors/temperatureout"
#define HUMIDITY_TOPIC 		"kitchen/sensors/humidityout"
#define VOLTAGE_TOPIC 		"kitchen/sensors/voltageout"
#define PRESSURE_TOPIC 		"kitchen/sensors/pressureout"

Adafruit_BME280 bme; // I2C
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
#define SEALEVELPRESSURE_HPA (1013.25)

#define DEBUG
#define DEBUGDELAY 60000

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long delayTime;

void getNextSample(float *Temperature,
				   float *Humidity,
				   float *Pressure)
{
	bme.takeForcedMeasurement();
	
	*Humidity    = bme.readHumidity();
	*Temperature = bme.readTemperature();
	//*Pressure    = bme.readPressure() / 100.0F;
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
		if (mqtt.connect("OutsideThermometer"))
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
	float temperature, humidity, pressure;

	digitalWrite(6UL, HIGH);
	getNextSample(&temperature, &humidity, &pressure);

	Serial.print("Humidity: 	");
	Serial.print(humidity);
	Serial.print(" %\r\n");

	Serial.print("Temperature: 	");
	Serial.print(temperature);
	Serial.print(" °C\r\n");

	//Serial.print("Pressure: 	");
	//Serial.print(pressure);
	//Serial.print(" mBar\r\n");
	Watchdog.reset();

	uint16 Vin = analogRead(A1);
	float Vout = ((double)Vin * VREF / BITS) * (R1 + R2) / R2;
	Serial.print("VBAT: 		");
	Serial.print(Vout);
	Serial.print(" V\r\n");

	connectWifi();
	connectMQTT();

	Watchdog.reset();

	Serial.print("Sending ... ");
	mqtt.publish(TEMPERATURE_TOPIC, String(temperature).c_str(), 	true);
	mqtt.publish(HUMIDITY_TOPIC, 	String(humidity).c_str(), 		true);
	mqtt.publish(VOLTAGE_TOPIC, 	String(Vout).c_str(), 			true);
	mqtt.publish(PRESSURE_TOPIC, 	String(pressure).c_str(), 		true);
	Serial.print("done\r\n");

	Watchdog.reset();

	disconnectMQTT();
	disconnectWifi();
	digitalWrite(6, LOW);
}

void loop() {

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
