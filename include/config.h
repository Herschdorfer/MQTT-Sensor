
/**
 * @brief MQTT Settings.
 */
#define MQTT_SERVER "192.168.0.1"
#define MQTT_SERVERPORT 3307
#define MQTT_USERNAME "test"
#define MQTT_PASSWORD "test"
#define WIFI_PASS "test"
#define WIFI_SSID "test"

/**
 * @brief MQTT Topic settings.
 */
#define TEMPERATURE_TOPIC "kitchen/sensors/temperatureout"
#define HUMIDITY_TOPIC "kitchen/sensors/humidityout"
#define VOLTAGE_TOPIC "kitchen/sensors/voltageout"

/**
 * @brief Timing settings.
 */
#define SENDING_INTERVAL 5  // Defines the intervall where data is send to MQTT
#define CMD_INTERVAL 500  // defines the intervall in which commands a retried.

/**
 * @brief DHT Library settings.
 */
#define DHT_PIN 2       // what digital pin we're connected to
#define DHT_TYPE DHT22  // DHT 11 is also possible

/**
 * @brief ADC battery reading settings.
 */
#define ANALOG_BITS 12
#define BITS 4096
#define V_REF 3.3  // Reference voltage
#define R1 456     // used for fine tuning
#define R2 978     // used for fine tuning