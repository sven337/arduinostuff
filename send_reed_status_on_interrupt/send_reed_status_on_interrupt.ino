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
static const int LED_ROUGE = 6;
static const int LED_JAUNE = 4;
static const int LED_BLEUE = 5;
static const int REED = 7;
static unsigned long pulse = 0;
static unsigned long old_pulse = 0;

// Define this to disable power down mode and spit out debug info on serial
//#define DEBUG_OUTPUT

#ifndef DEBUG_OUTPUT
#define printf (void)
#endif

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9, 8);

const uint64_t address_pi = 0xF0F0F0F0F0LL;

#ifndef DEBUG_OUTPUT
ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}
#endif

static void check_radio(void)
{
	bool tx, fail, rx;
	led(LED_BLEUE, true);
	radio.whatHappened(tx, fail, rx);
	uint32_t message_count = 0;

	// Have we successfully transmitted?
	if (tx) {
		printf("Send:OK\n\r");
		led(LED_ROUGE, false);
	}
	// Have we failed to transmit?
	if (fail) {
		printf("Send:Failed\n\r");
		led(LED_ROUGE, true);
	}

	// Did we receive a message?
	if (rx) {
		;
	}
	led(LED_BLEUE, false);
	radio.powerDown();
}

void reed_interrupt(void)
{
	led(LED_BLEUE, true);
	pulse++;
//	PCintPort::detachInterrupt(REED);
	delay(10);
}

static void led(int led, bool on)
{
	digitalWrite(led, on);
}

void setup(void)
{
#ifdef DEBUG_OUTPUT
	Serial.begin(57600);
	printf_begin();
#endif
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
	led(LED_BLEUE, false);
	radio.setRetries(15, 15);
	radio.setAutoAck(true);
	radio.setChannel(95);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);

	radio.openWritingPipe(address_pi);
	led(LED_JAUNE, false);

	radio.printDetails();
	attachInterrupt(1, check_radio, FALLING);
	PCintPort::attachInterrupt(REED, &reed_interrupt, FALLING);
	led(LED_ROUGE, false);

}

void radio_send_int(unsigned long data)
{
	led(LED_JAUNE, true);
	radio.powerUp();
	delayMicroseconds(5000);
	radio.startWrite(&data, sizeof(unsigned long));
	led(LED_JAUNE, false);
}

void loop(void)
{
	if (pulse != old_pulse) {
		// Pulse
		printf("Pulse number %d...\n", pulse);
		old_pulse = pulse;
		radio_send_int(old_pulse);
//		PCintPort::attachInterrupt(REED, &reed_interrupt, FALLING);
		led(LED_BLEUE, false);
	}
#ifndef DEBUG_OUTPUT
	Sleepy::powerDown();
#endif
}
