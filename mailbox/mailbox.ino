#include <SPI.h>
#include <JeeLib.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

// Define this to enable analog comparator IRQ
// if not, report light value every second over radio
#define ANALOG_COMPARATOR_IRQ 1

const int CE_PIN = 9;
const int CSN_PIN = 8;
const int BATTERY_PIN = A3;
const int LED_YELLOW = 5;
const int LED_RED = 4;

static unsigned long last_trigger_at;
boolean ignore_next_trigger; // when  re-enabling the analog comparator after a deep sleep, sometimes it seems to retrigger immediately even at 4AM with no ambient light. So cheat by ignoring it.

#if ANALOG_COMPARATOR_IRQ
// See http://www.gammon.com.au/forum/?id=11916 for analog comparator
const int COMPARATOR_LDR_PIN = 6; //P3D
// const int REF_PIN = 7; //P4D
#endif

const int LDR_PIN = A2; //P3A

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address = 0xF0F0F0F0F3LL;

volatile boolean triggered = false;
unsigned int triggered_counter = 0;

static unsigned long last_ping_at = 0;

int init_failed = 0;

#if ANALOG_COMPARATOR_IRQ
ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

#endif

ISR (ANALOG_COMP_vect)
{
	if (ignore_next_trigger) {
		ignore_next_trigger = false;
		return;
	}

	if (!triggered) {
		triggered = true;
		triggered_counter++;
	}

	last_trigger_at = millis();
	digitalWrite(LED_YELLOW, 1);
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

	if (!ok) {
		return -1;
	}

	return 0;
}


int get_light_level()
{
	return analogRead(LDR_PIN);
}

void radio_send_bat(uint16_t bat, uint8_t type)
{
	radio_send('B', type, (bat >> 8) & 0xFF, bat & 0xFF);
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

	if ((radio.getPALevel() != RF24_PA_MAX) ||
		(radio.getDataRate() != RF24_250KBPS) ||
		(radio.getCRCLength() != RF24_CRC_16) || 
		(radio.getChannel() != 95)) {
		// failed to initialize radio
		init_failed = 1;
	}


#if ANALOG_COMPARATOR_IRQ
	ADCSRB = 0;           // (Disable) ACME: Analog Comparator Multiplexer Enable
	ACSR =  /*_BV (ACI)     // (Clear) Analog Comparator Interrupt Flag
        | */_BV (ACIE)    // Analog Comparator Interrupt Enable
        | _BV (ACIS1)  // ACIS1, ACIS0 - trigger on rising edge
		| _BV (ACIS0);
	DIDR1 = _BV(AIN1D) | _BV(AIN0D); // Disable digital input  on D6/D7 
#endif

	// Light level sensor
	pinMode(LDR_PIN, INPUT);

	pinMode(LED_YELLOW, OUTPUT);
	pinMode(LED_RED, OUTPUT);
	digitalWrite(LED_YELLOW, 1);
	digitalWrite(LED_RED, 1);
	delay(200);
	digitalWrite(LED_RED, 0);
	delay(200);
	digitalWrite(LED_RED, 1);
	delay(200);
	digitalWrite(LED_RED, 0);
	delay(200);
	digitalWrite(LED_RED, 1);
	delay(200);
	digitalWrite(LED_RED, 0);
	digitalWrite(LED_YELLOW, 0);
}

void loop() 
{
	int red = 0;
	while (init_failed) {
		digitalWrite(LED_RED, red);
		red = !red;
		// Christmas tree if init failed
		digitalWrite(LED_YELLOW, 1);
		delay(100);
		digitalWrite(LED_YELLOW, 0);
		delay(100);
		digitalWrite(LED_YELLOW, 1);
		delay(100);
		digitalWrite(LED_YELLOW, 0);
		delay(100);
		return;
	}

	if (millis() > last_ping_at + 3600000) {
		uint16_t battery_level = analogRead(BATTERY_PIN);
		radio_send_bat(battery_level, 'N');
	}

	if (millis() - last_trigger_at > 5000) {
		digitalWrite(LED_YELLOW, 0);
		digitalWrite(LED_RED, 0);
	} 

	if (triggered) {
		printf("Triggered %d times, at %d seconds\n", triggered_counter, millis() / 1000);
		bool fail = true;
		int retries = 3;
		uint16_t battery_level = analogRead(BATTERY_PIN);

		while (retries-- && fail) {
			if (!radio_send('I', 'R', 'Q', 0)) {
				fail = false;
			}
			radio_send_bat(battery_level, 'N');
		}

		if (fail) { 
			digitalWrite(LED_RED, 1);
		} else {
			digitalWrite(LED_RED, 0);
			digitalWrite(LED_YELLOW, 1);
			delay(500);
			digitalWrite(LED_YELLOW, 0);
			delay(500);
			digitalWrite(LED_YELLOW, 1);
			delay(500);
			digitalWrite(LED_YELLOW, 0);
		}
		
		triggered = false;
		// Right after we were triggered, go to power down (not just idle) for ~12hours.
		Serial.flush();
		ACSR &= ~_BV(ACIE);
		Sleepy::loseSomeTime(32768);
		digitalWrite(LED_RED, 0);
		digitalWrite(LED_YELLOW, 0);
		int i = 1318;
//		int i = 131;
		while (i--) {
			Sleepy::loseSomeTime(32768);
		}
//		ignore_next_trigger = true; XXX does it end up ignoring legit triggers?
		ACSR |= _BV(ACIE);
	}


	Serial.flush();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_cpu();
	sleep_disable();
}



