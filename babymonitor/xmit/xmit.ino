// Based on ESP_MCP3201_SPI


#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "wifi_params.h"
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

WiFiUDP udp;
const int udp_target_port = 45990;
const IPAddress IP_target(192,168,0,2);

// Pin definitions: 
const int scePin = 15;   	// SCE - Chip select
/* HW definition of alternate function:
static const uint8_t MOSI  = 13;
static const uint8_t MISO  = 12;
static const uint8_t SCK   = 14;
*/
/*  Hardware: 
      MCP3201 Pin   ---------------- ESP8266 Pin
-       1-VREF      ----------------  3,3V
-       2-IN+       ----------------  ANALOG SIGNAL +
-       3-IN-       ----------------  ANALOG SIGNAL -
-       4-GND       ----------------  GND
-       5-CS        ----CS----------  GPIO15/CS (PIN 19)
-       6-Dout(MISO)----MISO--------  GPIO12/MISO (PIN 16)
-       7-CLK       ----SCLK--------  GPIO14 (PIN 17)
-       8-VDD       ----------------  3.3V  
*/

uint16_t adc_buf[2][700]; // ADC data buffer, double buffered
int current_adc_buf; // which data buffer is being used for the ADC (the other is being sent)
unsigned int adc_buf_pos; // position in the ADC data buffer
int send_samples_now; // flag to signal that a buffer is ready to be sent

void spiBegin(void) 
{
  pinMode(scePin, OUTPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
  digitalWrite(scePin, HIGH);
}

#define ICACHE_RAM_ATTR     __attribute__((section(".iram.text")))
static inline ICACHE_RAM_ATTR uint8_t transfer(uint8_t data) 
{
	while(SPI1CMD & SPIBUSY) {}

	SPI1W0 = data;
	SPI1CMD |= SPIBUSY;
	while(SPI1CMD & SPIBUSY) {}
	return (uint8_t) (SPI1W0 & 0xff);
}

static inline ICACHE_RAM_ATTR uint16_t transfer16(void) {
	union {
		uint16_t val;
		struct {
			uint8_t lsb;
			uint8_t msb;
		};
	} out;

	out.msb = transfer(0);
	out.lsb = transfer(0);
	return out.val;
}


void ICACHE_RAM_ATTR sample_isr(void)
{
	uint16_t val;

	// Read a sample from ADC
	digitalWrite(scePin, LOW);
	val = transfer16();
	digitalWrite(scePin, HIGH);
	adc_buf[current_adc_buf][adc_buf_pos] = val & 0xFFF;
	adc_buf_pos++;

	// If the buffer is full, signal it's ready to be sent and switch to the other one
	if (adc_buf_pos > sizeof(adc_buf[0])/sizeof(adc_buf[0][0])) {
		adc_buf_pos = 0;
		current_adc_buf = !current_adc_buf;
		send_samples_now = 1;
	}
}
 
void setup(void)
{ 
	Serial.begin(115200);
	Serial.println("I was built on " __DATE__ " at " __TIME__ "");

	WiFi.begin ( ssid, password );
	IPAddress myip(192, 168, 0, 32);
	IPAddress gw(192, 168, 0, 1);
	IPAddress subnet(255, 255, 255, 0);
	WiFi.config(myip, gw, subnet);
	Serial.print("Connecting to ");
	Serial.print(ssid); Serial.print(" "); Serial.print(password);
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

	ArduinoOTA.begin();

	spiBegin(); 
	
	timer1_isr_init();
	timer1_attachInterrupt(sample_isr);
	timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
	timer1_write(clockCyclesPerMicrosecond() / 16 * 80); //80us = 12.5kHz sampling freq
	Serial.println("setup done");
}

 
void loop() 
{
	ArduinoOTA.handle();
	if (send_samples_now) {
		udp.beginPacket(IP_target, udp_target_port);
		udp.write((const uint8_t *)(&adc_buf[!current_adc_buf][0]), sizeof(adc_buf[0]));
		udp.endPacket();
		send_samples_now = 0;
	}	  
}

