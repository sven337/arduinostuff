#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#define UPDATER_NO_MDNS
#include "ESP8266mDNS.h"
#include <ArduinoOTA.h>

#include "wifi_params.h"
#include "sounddata.h"


const int snd = 12;

void setup ( void ) {
	Serial.begin ( 57600 );
	Serial.println("setup");
	pinMode (snd, OUTPUT);
	analogWriteFreq(50000);
	analogWriteRange(256);

	Serial.println("done setup");
}

void loop ( void ) {
	static int idx = 0;
	idx++;
//	Serial.print("Writing ");
	analogWrite(snd, sounddata_data[idx]);
	delayMicroseconds(120);
//	Serial.println(sounddata_data[idx]);
	if (idx > sizeof(sounddata_data)/sizeof(sounddata_data[0])) {
		idx = 0;
	}
}
