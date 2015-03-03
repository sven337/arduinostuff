#include <PinChangeInt.h>

#include <SPI.h>
#include <JeeLib.h>
#include <Ports.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

/** FTDI 
  6 5
  4 7
 */
static const int LED_ROUGE = 15; // P2 A
static const int LED_JAUNE = 4;
static const int LED_BLEUE = 5;
static const int REED = 7;
static const int BATTERY_PIN = A2;
static uint16_t pulse = 0;
static uint16_t old_pulse = 0;

static unsigned long reed_interrupt_at = 0;
static unsigned long last_ping_at = 0;
static unsigned long transmit_failed_at = 0;

// Define this to disable power down mode and spit out debug info on serial

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9, 8);

const uint64_t address_pi = 0xF0F0F0F0F0LL;

ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

void (*reboot)(void) = 0;

static void check_radio(void)
{
	bool tx, fail, rx;
	radio.whatHappened(tx, fail, rx);
	uint32_t message_count = 0;

	// Have we successfully transmitted?
	if (tx) {
		printf("Send:OK\n\r");
		led(LED_ROUGE, false);
		transmit_failed_at = 0;
	}
	// Have we failed to transmit?
	if (fail) {
		printf("Send:Failed\n\r");
		led(LED_ROUGE, true);
		if (!transmit_failed_at)
			transmit_failed_at = millis();
	}

	// Did we receive a message?
	if (rx) {
		;
	}
	radio.powerDown();
}

void reed_interrupt(void)
{
	led(LED_BLEUE, true);
	pulse++;
	reed_interrupt_at = millis();
	delay(10);
}

static void led(int led, bool on)
{
	digitalWrite(led, on);
}

void radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
	uint8_t payload[4] = { p0, p1, p2, p3 };
	led(LED_JAUNE, true);
	radio.powerUp();
	delayMicroseconds(5000);
	radio.startWrite(&payload, 4);
	led(LED_JAUNE, false);
	last_ping_at = millis();
}


void radio_send_pulse(uint16_t pulse)
{
	radio_send('P', (pulse >> 8) & 0xFF, pulse & 0xFF, 0);
}

void radio_send_bat(uint16_t bat, uint8_t type)
{
	radio_send('B', type, (bat >> 8) & 0xFF, bat & 0xFF);
}

void setup(void)
{
	Serial.begin(57600);
	printf_begin();
	pinMode(REED, INPUT);
	digitalWrite(REED, true);
	pinMode(LED_ROUGE, OUTPUT);
	pinMode(LED_JAUNE, OUTPUT);
	pinMode(LED_BLEUE, OUTPUT);
	led(LED_ROUGE, true);
	led(LED_JAUNE, true);
	led(LED_BLEUE, true);

	radio.begin();
	radio.powerDown();
	radio.setRetries(15, 15);
	radio.setAutoAck(true);
	radio.setChannel(95);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);

	radio.openWritingPipe(address_pi);

	radio.printDetails();
	attachInterrupt(1, check_radio, FALLING);
	PCintPort::attachInterrupt(REED, &reed_interrupt, FALLING);
	delay(500);
	led(LED_ROUGE, false);
	delay(500);
	led(LED_JAUNE, false);
	delay(500);
	led(LED_BLEUE, false);

	radio_send_bat(analogRead(BATTERY_PIN), 'N');
}

void loop(void)
{
	if (pulse != old_pulse) {
		// Pulse
		printf("Pulse number %d...\n", pulse);
		old_pulse = pulse;

		radio_send_pulse(old_pulse);
		Sleepy::loseSomeTime(10000);
	}

	if (millis() > reed_interrupt_at + 3000) {
		led(LED_BLEUE, false);
	}

	Serial.flush();
	Sleepy::loseSomeTime(65535);
	Sleepy::loseSomeTime(65535);
	Sleepy::loseSomeTime(65535);
	if (millis() > last_ping_at + 600000L) {
		// Check battery level
		uint16_t battery_level = analogRead(BATTERY_PIN);
		if (battery_level < 560) {
			// Bat level < 3.6V = low bat
			radio_send_bat(battery_level, 'L');
		}

		// Ping receiver to confirm we're alive
		printf("Pinging at %lu\n", millis());
		radio_send_bat(battery_level, 'N');

		// If transmission has been failing for one hour, try to reboot
		if (transmit_failed_at && millis() - transmit_failed_at  > 3600000L) {
			reboot();
		}
	}
}
