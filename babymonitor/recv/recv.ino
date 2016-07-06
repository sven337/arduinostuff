#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "ESP8266mDNS.h"
#include <ArduinoOTA.h>

#include "wifi_params.h"


const int mySDA = D7;
const int mySCL = D6;

const int AMPLI_MUTE_PIN = D2;
const int AMPLI_SHUTDOWN_PIN = D1;

const int udp_recv_port = 45990;
	
const IPAddress myip(192, 168, 0, 31);

WiFiUDP udp;
TwoWire i2c;

#define NB_DATA_BUFS 5
uint16_t data_buf[NB_DATA_BUFS][700]; // data buffer, N buffered

unsigned int current_play_data_buf; // current data buf being played
unsigned int play_data_buf_pos; // position in the ADC data buffer
unsigned int current_recv_data_buf; // current data buf being received

bool play_waiting = true;
bool amplifier_stopped = false;
long play_waiting_at;

unsigned long I2C_total_time;

#include "brzo_i2c.h"

static inline ICACHE_RAM_ATTR uint8_t DAC(uint16_t value) 
{
	/* value is 76543210 XXXXBA98
	per the datasheet for fast write:
	1 1 0 0 A2 A1 A0 0 <ACK> 0 0 PD1 PD0 D11 D10 D9 D8 <ACK> D7 D6 D5 D4 D3 D2 D1 D0 <ACK> 
	*/
	// measured perf at 

	uint8_t buf[2] = { (value >> 8) & 0x0F, (value & 0xFF) };
	brzo_i2c_start_transaction(0x60, 800000);
	brzo_i2c_write(buf, 2, false);
	return brzo_i2c_end_transaction();
}

void ICACHE_RAM_ATTR playsample_isr(void)
{
	if (play_waiting) {
		return;
	}

	long now = micros();

	DAC(data_buf[current_play_data_buf][play_data_buf_pos]);
	play_data_buf_pos++;
	if (play_data_buf_pos >= sizeof(data_buf[0])/sizeof(data_buf[0][0])) {
		play_data_buf_pos = 0;
		current_play_data_buf++;
		Serial.print("I2C total time "); Serial.println(I2C_total_time);
		I2C_total_time = 0;
		if (current_play_data_buf == NB_DATA_BUFS) {
			current_play_data_buf = 0;
		}

		if (current_play_data_buf == current_recv_data_buf) {
			play_waiting = true;
			play_waiting_at = micros();
		}
	}

	I2C_total_time += micros() - now;
}

void ota_onstart(void)
{
	// Disable timer when an OTA happens
	timer1_detachInterrupt();
	timer1_disable();
}

void ota_onprogress(unsigned int sz, unsigned int total)
{
	Serial.print("OTA: "); Serial.print(sz); Serial.print("/"); Serial.print(total);
	Serial.print("="); Serial.print(100*sz/total); Serial.println("%");
}

void ota_onerror(ota_error_t err)
{
	Serial.print("OTA ERROR:"); Serial.println((int)err);
}


void setup ( void ) 
{
	Serial.begin ( 115200 );
	Serial.println("I was built on " __DATE__ " at " __TIME__ "");
	
/*	i2c.begin(mySDA, mySCL);
	i2c.setClock(400000);*/
	brzo_i2c_setup(mySDA, mySCL, 10);

	WiFi.mode(WIFI_STA);
	WiFi.begin ( ssid, password );
	IPAddress gw(192, 168, 0, 1);
	IPAddress subnet(255, 255, 255, 0);
	WiFi.config(myip, gw, subnet);
	Serial.print("Connecting to wifi");
	while ( WiFi.status() != WL_CONNECTED ) {
		delay ( 500 );
		Serial.print ( "." );
	}

	Serial.println ( "" );
	Serial.print ( "Cnnectd to " );
	Serial.println ( ssid );
	Serial.print ( "IP " );
	Serial.println ( WiFi.localIP() );

	ArduinoOTA.onStart(ota_onstart);
	ArduinoOTA.onError(ota_onerror);
	ArduinoOTA.onProgress(ota_onprogress);
	ArduinoOTA.setHostname("bb-recv");
	ArduinoOTA.begin();

	timer1_isr_init();
	timer1_attachInterrupt(playsample_isr);
	timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
	timer1_write(clockCyclesPerMicrosecond() / 16 * 50); //50us = 20 kHz sampling freq

	udp.begin(udp_recv_port);

	pinMode(AMPLI_MUTE_PIN, OUTPUT);
	pinMode(AMPLI_SHUTDOWN_PIN, OUTPUT);


}

void loop ( void ) 
{
	ArduinoOTA.handle();
	int sz = udp.parsePacket();
	if (sz) {
		udp.read((unsigned char *)&data_buf[current_recv_data_buf][0], sz);
		current_recv_data_buf++;
		if (current_recv_data_buf == NB_DATA_BUFS) {
			current_recv_data_buf = 0;
			if (current_recv_data_buf == current_play_data_buf && !play_waiting) {
				Serial.println("buffer overflow when receiving");
			}
		}
		if (play_waiting) {
			Serial.print("Restarting play, was waiting (us)"); Serial.println(micros() - play_waiting_at);
			// Re-enable *then* unmute in that order to avoid pops
			digitalWrite(AMPLI_SHUTDOWN_PIN, 1);
			digitalWrite(AMPLI_MUTE_PIN, 1);
			play_waiting = false;
			amplifier_stopped = false;
		}

		Serial.println("");
	}

	if (!amplifier_stopped && play_waiting) {
		if ((micros() - play_waiting_at) > 2000 * 1000) {
			// If nothing has been played for two seconds, shut down the amplifier 
			Serial.println("Shutting down amplifier!");
			digitalWrite(AMPLI_SHUTDOWN_PIN, 0);
			digitalWrite(AMPLI_MUTE_PIN, 0);
			amplifier_stopped = true;
		}
	}
}
