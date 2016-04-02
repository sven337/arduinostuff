#include <SPI.h>
#include <JeeLib.h>
#include <OneWire.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int CE_PIN = 9;
const int CSN_PIN = 8;
const int BATTERY_PIN = A3;
const int LED_YELLOW = 5;
const int LED_RED = 4;
const int DS18B20_PIN = 7;

OneWire ds(DS18B20_PIN);

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address = 0xF0F0F0F0F4LL;

static unsigned long last_ping_at = 0;

int init_failed = 0;

//const uint8_t thermometer_identification_letter = 'L'; // "living room"
const uint8_t thermometer_identification_letter = 'E'; // "exterior"
//const uint8_t thermometer_identification_letter = 'B'; // "bedroom"

ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

int radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
	uint8_t payload[4] = { p0, p1, p2, p3 };
	last_ping_at = millis();
	radio.powerUp();
	delayMicroseconds(5000);
	bool ok = radio.write(payload, 4);
	if (ok) 
		Serial.println("send ok");
	else
		Serial.println("send KO");
	
	radio.powerDown();

	if (!ok) {
		return -1;
	}

	return 0;
}


int radio_send_bat(uint16_t bat)
{
	return radio_send('B', thermometer_identification_letter, (bat >> 8) & 0xFF, bat & 0xFF);
}

void setup(){
	printf_begin();
	Serial.begin(57600);

	// Radio init
	radio.begin();
	radio.powerDown();
	radio.setRetries(15, 15);
	radio.setChannel(95);
	radio.setCRCLength(RF24_CRC_16);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
 	radio.setAutoAck(true);
	radio.openWritingPipe(pipe_address);
	radio.openReadingPipe(1, pipe_address);

	radio.printDetails();

	if ((radio.getDataRate() != RF24_250KBPS) ||
		(radio.getCRCLength() != RF24_CRC_16) || 
		(radio.getChannel() != 95)) {
		// failed to initialize radio
		init_failed = 1;
	}


	pinMode(LED_YELLOW, OUTPUT);
	pinMode(LED_RED, OUTPUT);
	digitalWrite(LED_YELLOW, 1);
	digitalWrite(LED_RED, 1);
	delay(100);
	digitalWrite(LED_RED, 0);
	delay(100);
	digitalWrite(LED_RED, 1);
	delay(100);
	digitalWrite(LED_RED, 0);
	delay(100);
	digitalWrite(LED_RED, 1);
	delay(100);
	digitalWrite(LED_RED, 0);
	digitalWrite(LED_YELLOW, 0);
}

float battery_voltage(uint16_t battery_level)
{
	float lvl = (float)battery_level * 3.3 / 65536.0;
	return lvl;
}

void loop() 
{
	int red = 0;
	while (init_failed) {
			digitalWrite(LED_RED, red);
			red = !red;
			// Christmas tree if init failed
			digitalWrite(LED_YELLOW, 1);
			delay(50);
			digitalWrite(LED_YELLOW, 0);
			delay(50);
			digitalWrite(LED_YELLOW, 1);
			delay(50);
			digitalWrite(LED_YELLOW, 0);
			delay(50);
	}

	uint16_t battery_level = analogRead(BATTERY_PIN);
	bool fail = radio_send_bat(battery_level);

	if (fail) {
		digitalWrite(LED_RED, 1);
	}

	uint8_t addr[8];
	uint8_t data[12];
	int i = 4;
	while(!ds.search(addr) && i--) {
		Serial.println("No more addresses.");
		ds.reset_search();
		delay(250);
	}
    if (!i) {
		return;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44, 1);
	delay(800);
	uint8_t present = ds.reset();
	ds.select(addr);    
	ds.write(0xBE);

	Serial.print("  Data = ");
	Serial.print(present, HEX);
	Serial.print(" ");
	for (int i = 0; i < 9; i++) {
		data[i] = ds.read();
		Serial.print(data[i], HEX);
		Serial.print(" ");
	}
	Serial.println();

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

	i = 100;

	while (i--) {
		if (!radio_send('T', thermometer_identification_letter, (raw >> 8) & 0xFF, raw & 0xFF)) {
			break;
		}

		Serial.println("radio message not sent, not sleeping");
		delay(2000);
	}
	
	Serial.println("going to sleep now");
	Serial.flush();
	Sleepy::loseSomeTime(32768L);
	Serial.println("waking up");
	digitalWrite(LED_RED, 0);

	for (int i = 1; i < 15L*60L*1000L / 32768L; i++) {
		if (battery_voltage(battery_level) >= 1.4f) {
			// Battery is overcharged, try to waste energy!
			delay(10000);
		}
		if (!Sleepy::loseSomeTime(32768L)) {
			Serial.println("woken up by intr");
		}
	}
}



