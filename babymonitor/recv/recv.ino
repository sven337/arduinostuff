#include <Wire.h>
#define USE_US_TIMER
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#define UPDATER_NO_MDNS
#include "ESP8266mDNS.h"
#include <ArduinoOTA.h>

#include "wifi_params.h"
#define ICACHE_RODATA_ATTR  __attribute__((section(".irom.text")))

#include "sounddata.h"

const int mySDA = 4;
const int mySCL = 5;

TwoWire i2c;

uint32_t usToTicks(uint32_t us)
{
	return (clockCyclesPerMicrosecond() / 16 * us);     // converts microseconds to tick
}

void playsample_isr(void)
{
	/*static uint16_t value = 0;
	DAC(value++);
	if (value == (1 << 12)) {
		value = 0;
	}*/
	static int idx = 0;
	DAC(sounddata_data[idx++]);
	if (idx > sizeof(sounddata_data)/sizeof(sounddata_data[0])) {
		idx = 0;
	}
	
}

void DAC(uint16_t value) 
{
	i2c.beginTransmission(0x60); // address is 0x60
	i2c.write(0x40); // command is 0x40
	i2c.write((value >> 4) & 0xFF);
	i2c.write((value & 0x0F) << 4);
	i2c.endTransmission();
}

void setup ( void ) {
	Serial.begin ( 74880 );
	Serial.println("setup");
	i2c.begin(mySDA, mySCL);
	i2c.setClock(400000);

	Serial.println("done setup");

	timer1_isr_init();
	timer1_attachInterrupt(playsample_isr);
	timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
	timer1_write(clockCyclesPerMicrosecond() / 16 * 80);
}

void loop ( void ) {
}
