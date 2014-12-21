#include <SPI.h>
#include <JeeLib.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

// Define this to enable analog comparator IRQ
// if not, report light value every second over radio
#define ANALOG_COMPARATOR_IRQ 1

const int CE_PIN = 8;
const int CSN_PIN = 9;

#if ANALOG_COMPARATOR_IRQ
// See http://www.gammon.com.au/forum/?id=11916 for analog comparator
const int LDR_PIN = 6; //P3D
// const int REF_PIN = 7; //P4D
#else
const int LDR_PIN = A2; //P3A
#endif

const int NOTIFY_LED_PIN = 5; //P2D

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address = 0xF0F0F0F0F3LL;

volatile boolean triggered = false;
unsigned int triggered_counter = 0;

static long next_ldr_report_at;

#if ANALOG_COMPARATOR_IRQ
ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

#endif

ISR (ANALOG_COMP_vect)
  {
  triggered = true;
  triggered_counter++;

  // debounce
  ACSR &= ~_BV(ACIE);
  }


void radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
	uint8_t payload[4] = { p0, p1, p2, p3 };
	radio.stopListening();
	delay(10);
	radio.powerUp();
	bool ok = radio.write(payload, 4);
	if (ok) 
		Serial.println("send ok");
	else
		Serial.println("send KO");
	delay(10);
	radio.startListening();
	delay(10);
}


int get_light_level()
{
	return analogRead(LDR_PIN);
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

#if ANALOG_COMPARATOR_IRQ
	ADCSRB = 0;           // (Disable) ACME: Analog Comparator Multiplexer Enable
	ACSR =  _BV (ACI)     // (Clear) Analog Comparator Interrupt Flag
        | _BV (ACIE)    // Analog Comparator Interrupt Enable
        | _BV (ACIS1)  // ACIS1, ACIS0 - trigger on rising edge
		| _BV (ACIS0);
#else
	// Light level sensor
	pinMode(LDR_PIN, INPUT);
#endif

}

void loop() 
{

#if ANALOG_COMPARATOR_IRQ
	if (triggered) {
		printf("Triggered %d times, at %d seconds\n", triggered_counter, millis() / 1000);
		radio_send('I', 'R', 'Q', 0);
		radio_send('I', 'R', 'Q', 0);
		radio_send('I', 'R', 'Q', 0);
		radio_send('I', 'R', 'Q', 0);
		triggered = false;
		delay(100);
		ACSR |= _BV(ACIE);
	}

	while (radio.available()) {
		uint8_t payload[4];
		radio.read(payload, 4);
		switch (payload[0]) {
			case 'Q': 
				{
				int light_level = get_light_level();
				radio_send('L', 'N', light_level & 0xFF, (light_level >> 8) & 0xFF); 
				}
			break;
		}
	}

//	Sleepy::powerDown();
#else
	if (millis() > next_ldr_report_at) {
		next_ldr_report_at = millis() + 1000;
		int light_level = get_light_level();
		printf("Mailbox light %d\n", light_level);
		radio_send('L', 'N', light_level & 0xFF, (light_level >> 8) & 0xFF); 
	}
#endif


}



