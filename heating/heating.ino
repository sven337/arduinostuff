#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include "wifi_params.h"

ESP8266WebServer websrv (80);
WiFiUDP udp;
const int udp_port = 2222;

const int chaudiere = 14;
const int pushbtn = 12;

int pwm;

long int last_reboot_at;

ArduinoOTA otasrv("heater-", 8266, true);

void handleRoot() {
	char temp[1024];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	int reboot_sec = last_reboot_at / 1000;
	int reboot_min = reboot_sec / 60;
	int reboot_hour = reboot_min / 60;
	snprintf ( temp, 1024,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='30'/>\
    <title>Regulation chaudiere ESP8266</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from ESP8266!</h1>\
    <p>Built on %s at %s</p><p>Uptime: %02d:%02d:%02d</p><p>Last reboot at %02d:%02d:%02d timestamp</p>\
	<p>Current power is %d</p> \
  </body>\
</html>",

		__DATE__, __TIME__, hr, min % 60, sec % 60, reboot_hour, reboot_min % 60, reboot_sec % 60,
		pwm
	);
	websrv.send ( 200, "text/html", temp );
}

void handleNotFound() {
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += websrv.uri();
	message += "\nMethod: ";
	message += ( websrv.method() == HTTP_GET ) ? "GET" : "POST";
	message += "\nArguments: ";
	message += websrv.args();
	message += "\n";

	for ( uint8_t i = 0; i < websrv.args(); i++ ) {
		message += " " + websrv.argName ( i ) + ": " + websrv.arg ( i ) + "\n";
	}

	websrv.send ( 404, "text/plain", message );
}

void setup ( void ) {
	pinMode ( chaudiere, OUTPUT );
	pinMode(pushbtn, INPUT_PULLUP);
	digitalWrite(chaudiere, 1);
	Serial.begin ( 115200 );
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
	Serial.print ( "Connected to " );
	Serial.println ( ssid );
	Serial.print ( "IP address: " );
	Serial.println ( WiFi.localIP() );

	udp.begin(udp_port);
	websrv.on ( "/", handleRoot );
	websrv.on ( "/inline", []() {
		websrv.send ( 200, "text/plain", "this works as well" );
	} );
	websrv.onNotFound ( handleNotFound );
	websrv.begin();
	Serial.println ( "HTTP websrv started" );

	otasrv.setup();

	last_reboot_at = millis();
}

void udp_send(const char *str)
{
	udp.beginPacket(udp.remoteIP(), udp.remotePort());
	udp.write(str);
	udp.endPacket();
}

void change_pwm(const char *buf)
{
	char str[255];
	int duty_cycle = atoi(buf);
	Serial.print("Received UDP request to set duty cycle to ");
	Serial.println(duty_cycle);

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
		sprintf(str, "Requested invalid duty cycle %d\n", duty_cycle);
		udp_send(str);
	}

}

void udp_send_status_report()
{
	char str[255];
	sprintf(str, "Nothing to report");
	udp_send(str);
}

void parse_cmd(const char *buf)
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
	otasrv.handle();
	websrv.handleClient();

	if (!digitalRead(pushbtn)) {
		while(!digitalRead(pushbtn)) ;

		if (pwm) { 
			change_pwm("0");
		} else {
			change_pwm("100");
		}
	}
	char packetBuffer[255];
	int packetSize = udp.parsePacket();
	if (packetSize) {
		Serial.print("Received packet of size ");
		Serial.print(packetSize);
		Serial.print(" from ");
		IPAddress remoteIp = udp.remoteIP();
		Serial.print(remoteIp);
		Serial.print(", port ");
		Serial.println(udp.remotePort());

		// read the packet into packetBufffer
		int len = udp.read(packetBuffer, 255);
		if (len > 0) {
			packetBuffer[len] = 0;
		}
		Serial.print("Contents: ");
		Serial.println(packetBuffer);

		if (strncmp(packetBuffer, appcode, strlen(appcode))) {
			// Packet doesn't have security code, ignore it.
			udp.flush();
		} else {
			parse_cmd(&packetBuffer[strlen(appcode)]);
		}
	}
}

