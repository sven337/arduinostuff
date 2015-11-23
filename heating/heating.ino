#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#define UPDATER_NO_MDNS
#include <ArduinoOTA.h>

#include "wifi_params.h"

ESP8266WebServer websrv (80);
WiFiUDP udp;
const int udp_port = 2222;

const int chaudiere = 14;
const int pushbtn = 12;

int pwm;

unsigned long int forced_heating_until = 0;

static void handleRoot() {
	char temp[1024];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf ( temp, 1024,

"<html><head><title>ESP8266</title></head><body>\
    <h1>Hello from ESP8266!</h1>\
    <p>Built on %s at %s</p><p>Uptime: %02d:%02d:%02d = %d ms</p>\
	<p>Current power is %d, %sforced until %d</p> \
  </body></html>",

		__DATE__, __TIME__, hr, min % 60, sec % 60, millis(),
		pwm, (millis() < forced_heating_until ? "" : "not ", forced_heating_until)
	);
	websrv.send ( 200, "text/html", temp );
}

int analogRead(uint8_t pin)
{
}

void setup ( void ) {
	pinMode ( chaudiere, OUTPUT );
	pinMode(pushbtn, INPUT_PULLUP);
	digitalWrite(chaudiere, 1);
	Serial.begin ( 78400 );
	WiFi.begin ( ssid, password );
	IPAddress myip(192, 168, 0, 33);
	IPAddress gw(192, 168, 0, 254);
	IPAddress subnet(255, 255, 255, 0);
	WiFi.config(myip, gw, subnet);
	Serial.println ( "" );

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

	udp.begin(udp_port);
	websrv.on ( "/", handleRoot );

	websrv.begin();
	ArduinoOTA.begin();
}

void udp_send(const char *str)
{
	udp.beginPacket(udp.remoteIP(), udp.remotePort());
	udp.write(str);
	udp.endPacket();
}

void change_pwm(const void *buf)
{
	char str[255];
	int duty_cycle = atoi((const char *)buf);
	Serial.print("Recvd UDP req to set duty cycle to ");
	Serial.println(duty_cycle);

	if (millis() < forced_heating_until) {
		return;
	}

	if (duty_cycle >= 0 && duty_cycle <= 100) {
		pwm = duty_cycle;
		sprintf(str, "Setting duty cycle to %d\n", duty_cycle);
		udp_send(str);
		if (duty_cycle == 100) {
			digitalWrite(chaudiere, 0);
		} else {
			digitalWrite(chaudiere, 1);
		}
	} else {
		sprintf(str, "Req.invalid duty cycle %d\n", duty_cycle);
		udp_send(str);
	}

}

static void udp_send_status_report(void)
{
	char str[50];
	sprintf(str, "PWM %d", pwm);
	udp_send(str);
}

static void parse_cmd(const char *buf)
{
	if (!strncmp(buf, "PWM ", 4)) {
		buf += 4;
		change_pwm(buf);
	} else if (!strncmp(buf, "STATUS", 6)) {
		udp_send_status_report();
	} else if (!strncmp(buf, "ON", 2)) {
		change_pwm("100");
	} else if (!strncmp(buf, "OFF", 3)) {
		change_pwm("0");
	} 
}

void loop ( void ) {
	ArduinoOTA.handle();
	websrv.handleClient();

	if (!digitalRead(pushbtn)) {
		while(!digitalRead(pushbtn)) ;
		
		change_pwm("100");

		// Force heating for 20 minutes
		forced_heating_until = millis() + 20 * 60 * 1000; 

		// On overflow, put burner at full power, but don't set 
		// a end date, so the next change request will be accepted
		if (forced_heating_until < millis()) {
			forced_heating_until = 0;
		}

	}

	if (millis() > forced_heating_until) {
		forced_heating_until = 0;
	}

	char packetBuffer[255];
	int packetSize = udp.parsePacket();
	if (packetSize) {
		// read the packet into packetBufffer
		int len = udp.read(packetBuffer, 255);
		if (len > 0) {
			packetBuffer[len] = 0;
		}

		parse_cmd(&packetBuffer[0]);
	}
}

