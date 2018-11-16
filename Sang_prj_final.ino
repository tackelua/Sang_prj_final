/*
	Name:       project_final
	Created:	11/15/2018 9:28:14 AM
	Author:     GITH\tacke
	Version:	0.1
*/

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Button.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <TSL2561.h>

#define DEBUG	Serial
#define HUB_ID	"ESP_GITH"

#define ON HIGH
#define OFF LOW

bool RELAY_STT = false;
#define P_RELAY D5
#define P_BUTTON D3
#define P_DHT11	D6

const char* mqtt_server = "mic.duytan.edu.vn";
const char* mqtt_user = "Mic@DTU2017";
const char* mqtt_password = "Mic@DTU2017!@#";
const uint16_t mqtt_port = 1883;
WiFiClient mqtt_espClient;
PubSubClient mqtt_client(mqtt_espClient);
String tp_status = HUB_ID "/" "status";
String tp_control = HUB_ID "/" "control";
String tp_interval = HUB_ID "/" "interval";
String tp_relay = HUB_ID "/" "relay";
String tp_temperature = HUB_ID "/" "temperature";
String tp_humidity = HUB_ID "/" "humidity";
String tp_tsl = HUB_ID "/" "light";

bool dht_active = false;
bool dht_idx = true;
DHT dht(P_DHT11, DHT11);
float temperature;
float humidity;

uint32_t lux;
TSL2561 tsl(TSL2561_ADDR_FLOAT);

uint8_t puEnable = false, invert = true;
uint32_t dbTime = 50;
Button button(P_BUTTON, puEnable, invert, dbTime);

unsigned long time_update_sensor = 5000;

void led_on() {
	digitalWrite(LED_BUILTIN, LOW);
}
void led_off() {
	digitalWrite(LED_BUILTIN, HIGH);
}

void wifi_connect() {
	WiFiManager wifiManager;
	wifiManager.autoConnect(HUB_ID);
	WiFi.waitForConnectResult();
	if (WiFi.localIP() == INADDR_NONE) {
		DEBUG.println("Can not get IP. Restart");
		ESP.restart();
		delay(1000);
	}
}


bool mqtt_publish(String topic, String payload, bool retain) {
	if (!mqtt_client.connected()) {
		return false;
	}
	led_on();
	DEBUG.print(("MQTT<<<  "));
	DEBUG.println(topic);
	DEBUG.println(payload);
	DEBUG.println();

	bool ret = mqtt_client.publish(topic.c_str(), payload.c_str(), retain);
	led_off();
	yield();
	return ret;
}
void mqtt_reconnect() {
	if (!mqtt_client.connected()) {
		DEBUG.println(("\r\nAttempting MQTT connection..."));
		if (mqtt_client.connect(HUB_ID, mqtt_user, mqtt_password), tp_status, "offline") {
			DEBUG.println(("Connected."));
			mqtt_publish(tp_status, "online", true);
			mqtt_client.subscribe(tp_interval.c_str());
			mqtt_client.subscribe(tp_control.c_str());
		}
		else {
			DEBUG.print(("failed, rc="));
			DEBUG.println(String(mqtt_client.state()));
			return;
		}
	}
}
void mqtt_callback(char* topic, uint8_t* payload, unsigned int length) {
	String pl;
	led_on();
	for (uint i = 0; i < length; i++) {
		pl += (char)payload[i];
	}
	led_off();
	DEBUG.println(pl);
	String tp = topic;
	if (tp == tp_control) {
		if (pl == "on") {
			relay_on();
		}
		if (pl == "off") {
			relay_off();
		}
	}
	if (tp == tp_interval) {
		int interval = pl.toInt();
		if (interval > 0) {
			time_update_sensor = interval * 1000;
			DEBUG.println("time_update_sensor = " + String(time_update_sensor));
		}
	}
}
void mqtt_init() {
	mqtt_client.setServer(mqtt_server, mqtt_port);
	mqtt_client.setCallback(mqtt_callback);
}
void mqtt_loop() {
	if (!WiFi.isConnected()) {
		return;
	}
	if (!mqtt_client.connected()) {
		mqtt_reconnect();
	}
	mqtt_client.loop();
	yield();
}


bool dht_readTemperature() {
	if (!dht_active) {
		dht.begin();
	}
	float t = dht.readTemperature();
	if (isnan(t)) {
		DEBUG.println("Failed to read temperature from DHT sensor!\r\n");
		dht_active = false;
		return false;
	}
	temperature = t;
	dht_active = true;
	return true;
}
bool dht_readHumidity() {
	if (!dht_active) {
		dht.begin();
	}
	float t = dht.readTemperature();
	float h = dht.readHumidity();
	if (isnan(t) || isnan(h)) {
		DEBUG.println("Failed to read humidity from DHT sensor!\r\n");
		dht_active = false;
		return false;
	}
	temperature = t;
	humidity = h;
	dht_active = true;
	return true;
}
void dht_readTemperature(unsigned long ms) {
	static unsigned long t_last_read = millis();
	static unsigned long t_last_success = millis();
	static bool success = true;
	if (!success && (millis() - t_last_read < 2000)) {
		return;
	}
	if (millis() - t_last_success > ms) {
		success = dht_readTemperature();
		t_last_read = millis();
		if (success) {
			t_last_success = millis();
			dht_idx = false;
			String dht_temperature = String(temperature);
			DEBUG.println("TEMPERATURE: " + dht_temperature);
			mqtt_publish(tp_temperature, dht_temperature, false);
		}
	}
}
void dht_readHumidity(unsigned long ms) {
	static unsigned long t_last_read = millis();
	static unsigned long t_last_success = millis();
	static bool success = true;
	if (!success && (millis() - t_last_read < 2000)) {
		return;
	}
	if (millis() - t_last_success > ms) {
		success = dht_readHumidity();
		t_last_read = millis();
		if (success) {
			dht_idx = true;
			t_last_success = millis();
			String dht_humidity = String(humidity);
			DEBUG.println("HUMIDITY: " + dht_humidity);
			mqtt_publish(tp_humidity, dht_humidity, false);
		}
	}
}
void dht_read(unsigned long ms) {
	if (dht_idx) {
		dht_readTemperature(ms / 2);
	}
	else {
		dht_readHumidity(ms / 2);
	}
}

bool tsl_init() {
	if (tsl.begin()) {
		tsl.setGain(TSL2561_GAIN_16X);
		tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);
		DEBUG.println("Found TSL sensor");
		return true;
	}
	else {
		DEBUG.println("No TSL sensor");
		return false;
	}
}
bool tsl_read() {
	static bool tsl_active = false;
	if (!tsl_active) {
		tsl_active = tsl_init();
	}
	if (tsl_active) {
		uint32_t lum = tsl.getFullLuminosity();
		uint16_t ir, full;
		ir = lum >> 16;
		full = lum & 0xFFFF;
		lux = tsl.calculateLux(full, ir);
		return true;
	}
	return false;
}
void tsl_read(unsigned long ms) {
	static unsigned long t_last_read = millis();
	static unsigned long t_last_success = millis();
	static bool success = true;
	if (!success && (millis() - t_last_read < 2000)) {
		return;
	}
	if (millis() - t_last_success > ms) {
		success = tsl_read();
		t_last_read = millis();
		if (success) {
			t_last_success = millis();
			String tsl_data = String(lux);
			DEBUG.println("LIGHT: " + tsl_data);
			mqtt_publish(tp_tsl, tsl_data, false);
		}
	}
}

void relay_on() {
	RELAY_STT = true;
	digitalWrite(P_RELAY, ON);
	mqtt_publish(tp_relay, "on", true);
}
void relay_off() {
	RELAY_STT = false;
	digitalWrite(P_RELAY, OFF);
	mqtt_publish(tp_relay, "off", true);
}

void button_handle() {
	button.read();
	if (button.wasReleased()) {
		RELAY_STT = !RELAY_STT;
		if (RELAY_STT) {
			relay_on();
		}
		else {
			relay_off();
		}
	}
	if (button.pressedFor(5000)) {
		WiFi.disconnect();
		DEBUG.println("WiFi disconnect\r\nRestart");
		ESP.restart();
		delay(1000);
	}
}

void setup()
{
	DEBUG.begin(115200);
	DEBUG.println();
	WiFi.printDiag(DEBUG);
	pinMode(P_RELAY, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	led_on();
	relay_off();
	wifi_connect();
	mqtt_init();
	led_off();
}

void loop()
{
	mqtt_loop();
	button_handle();
	dht_readTemperature(time_update_sensor);
	dht_readHumidity(time_update_sensor);
	tsl_read(time_update_sensor);
}
