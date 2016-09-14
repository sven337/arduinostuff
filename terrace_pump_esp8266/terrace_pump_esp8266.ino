// ESP8266 terrace pump controller & exterior thermometer
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "wifi_params.h"
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <ESP8266HTTPClient.h>

const IPAddress my_addr(192, 168, 0, 15);

const int FORCE_PUMPING_PIN = 5;
const int FORCE_STOP_PUMPING_PIN = 4;
const int PUMP_CONTROL_PIN = 13;

const int DS18B20_PIN = 2;

bool is_pumping = 0;
unsigned long pump_until;

ESP8266WebServer websrv (80);
WiFiUDP udp;

float temperature;
unsigned long int send_temperature_at = 1000;

OneWire ds(DS18B20_PIN);

static void handleRoot() {
	char temp[1024];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf ( temp, 1024,

"<html><head><title>Pompe terrasse</title></head><body>\
    <h1>Bonjour !</h1>\
    <p>Built on %s at %s</p><p>Uptime: %02d:%02d:%02d = %d ms</p>\
	<p>Current temperature is %f C\n</p>\
    <p>Currently pumping: %d\n</p>	\
  </body></html>",

		__DATE__, __TIME__, hr, min % 60, sec % 60, (int)millis(),
		temperature, is_pumping
	);
	websrv.send ( 200, "text/html", temp );
	
	// Update temperature next round
	send_temperature_at = millis();
}

void start_pumping()
{
	if (!is_pumping) {
		is_pumping = 1;
		digitalWrite(PUMP_CONTROL_PIN, 1);
		pump_until = millis() + (long)3600*1000;
		printf("Starting pump... now %ld, until %ld\n", millis(), pump_until);
	}
}

void stop_pumping()
{
	if (is_pumping) {
		Serial.println("Stopping pump\n");
		is_pumping = 0;
		digitalWrite(PUMP_CONTROL_PIN, 0);
	}
}

void ota_onstart(void)
{
	stop_pumping();
}

void ota_onprogress(unsigned int sz, unsigned int total)
{
	Serial.print("OTA: "); Serial.print(sz); Serial.print("/"); Serial.print(total);
	Serial.print("="); Serial.print(100*sz/total); Serial.println("%%");
}

void ota_onerror(ota_error_t err)
{
	Serial.print("OTA ERROR:"); Serial.println((int)err);
}


void setup()
{
	Serial.begin(115200);
	Serial.println("I was built on " __DATE__ " at " __TIME__ "");
	
	pinMode(FORCE_PUMPING_PIN, INPUT_PULLUP);
	pinMode(FORCE_STOP_PUMPING_PIN, INPUT_PULLUP);
	pinMode(PUMP_CONTROL_PIN, OUTPUT);

	digitalWrite(PUMP_CONTROL_PIN, 0);
	WiFi.mode(WIFI_STA);
	WiFi.config(my_addr, IPAddress(192, 168, 127, 127), INADDR_NONE, INADDR_NONE);
	WiFi.begin(ssid, password);
	Serial.print("Connecting to wifi");
	// Wait for connection
	while ( WiFi.status() != WL_CONNECTED ) {
		delay ( 500 );
		Serial.print ( "." );
	}

	Serial.println ( "" );
	Serial.print ( "Cnnectd to " );
	Serial.println ( ssid );
	Serial.print ( "IP " );
	Serial.println ( WiFi.localIP() );

	websrv.on ( "/", handleRoot );
	websrv.begin();

	ArduinoOTA.onStart(ota_onstart);
	ArduinoOTA.onError(ota_onerror);
	ArduinoOTA.onProgress(ota_onprogress);
	ArduinoOTA.setHostname("terrace-pump");
	ArduinoOTA.begin();


	Serial.println("ready");
}

static void send_temperature_update()
{
	HTTPClient http;
	char URI[200];

	sprintf(URI, "http://192.168.0.6:5000/update/temperature/exterior/%f", temperature);

	http.begin(URI); 

	int httpCode = http.GET();

	// httpCode will be negative on error
	if(httpCode) {
		// HTTP header has been send and Server response header has been handled
		Serial.printf("[HTTP] GET... code: %d\n", httpCode);
	}

	http.end();
}


static bool web_boolean_req(const char *addr)
{
	HTTPClient http;
	char URI[150];

	sprintf(URI, "http://192.168.0.6:5000/%s/", addr);
	http.begin(URI); 

	int httpCode = http.GET();

	if (httpCode == HTTP_CODE_OK) {
		// HTTP header has been send and Server response header has been handled
		Serial.printf("[HTTP] GET... code: %d\n", httpCode);
		String payload = http.getString();
		Serial.println(payload);

		if (payload.startsWith("1")) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}

	http.end();
}

static bool must_do_deep_sleep()
{
	return web_boolean_req("deep_sleep_mode");
}

static bool web_check_pumping_requested()
{
	return web_boolean_req("terrace_pumping_request");
}

void loop()
{
	bool pumping_requested_by_button = !digitalRead(FORCE_PUMPING_PIN);
	bool pumping_stop_requested = !digitalRead(FORCE_STOP_PUMPING_PIN);
	
	ArduinoOTA.handle();
	websrv.handleClient();

	if (pumping_requested_by_button) {
		start_pumping();
	} 

	if (pumping_stop_requested) {
		stop_pumping();
	}

	if (millis() > pump_until) {
		stop_pumping();
	}
	
	if (millis() > send_temperature_at) {
		send_temperature_at = millis() + 5L * 60L * 1000L;
		uint8_t addr[8];
		uint8_t data[12];
		int i = 4;
		while(!ds.search(addr) && i--) {
			Serial.println("No more addresses.");
			ds.reset_search();
			delay(250);
		}
		
		if (i == -1) {
			Serial.println("probe error");
			// Notify error
			temperature = 999;
			send_temperature_update();
			return;
		}

		ds.reset();
		ds.select(addr);
		ds.write(0x44, 1);
		delay(800);
		uint8_t present = ds.reset();
		ds.select(addr);    
		ds.write(0xBE);

		for (int i = 0; i < 9; i++) {
			data[i] = ds.read();
		}

		// Convert the data to actual temperature
		// because the result is a 16 bit signed integer, it should
		// be stored to an "int16_t" type, which is always 16 bits
		// even when compiled on a 32 bit processor.
		int16_t raw = (data[1] << 8) | data[0];
		byte cfg = (data[4] & 0x60);
		// at lower res, the low bits are undefined, so let's zero them
		if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
		else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
		else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
		//// default is 12 bit resolution, 750 ms conversion time
		printf("Temperature is %d\n", (int)(100.0*(float)raw/16.0));
		temperature = raw/16.0;

		send_temperature_update();
		bool start_pump = web_check_pumping_requested();
	
		if (!start_pump) {
			stop_pumping();
		} else {
			start_pumping();
		}
		// Call my server to check if we need to deep sleep or not. This is used to enable OTAs from my desk. :)
		if (!start_pump && must_do_deep_sleep()) {
			Serial.println("deepsleep go");	
			ESP.deepSleep(5000*60000L); 
		}
	}

	delay(25);
}
