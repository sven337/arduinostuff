#define USE_US_TIMER
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#define UPDATER_NO_MDNS
#include "ESP8266mDNS.h"
#include <ArduinoOTA.h>
#include <Ticker.h>

#include "wifi_params.h"
#include "sounddata.h"

const int snd = 12;

uint32_t usToTicks(uint32_t us)
{
	return (clockCyclesPerMicrosecond() / 16 * us);     // converts microseconds to tick
}

void playsample_isr(void)
{
	static int idx = 0;
	analogWrite(snd, sounddata_data[idx++]);
	if (idx > sizeof(sounddata_data)/sizeof(sounddata_data[0])) {
		idx = 0;
	}
}
void setup ( void ) {
	Serial.begin ( 57600 );
	Serial.println("setup");
	pinMode (snd, OUTPUT);
	analogWriteFreq(50000);
	analogWriteRange(256);

	Serial.println("done setup");

	timer1_isr_init();
	timer1_attachInterrupt(playsample_isr);
	timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
	timer1_write(usToTicks(1000000/8000));
}

void loop ( void ) {
	;
}
