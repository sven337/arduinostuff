#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int LED_DIM_PIN = 3; // P3 analog 0
const int CE_PIN = 8;
const int CSN_PIN = 9;


#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xF0F0F0F0F2LL;

float bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (exp(0.6931471805599453 * in) - 1.0);
}

void set_led(float val)
{
	val = bri(val);
	analogWrite(LED_DIM_PIN, val * 255);
	printf("Set LED to %d%%\n", (int)(val*100));
}

void strobe()
{
	int i = 1000;

	while (i--) {
		set_led(1.0);
		delay(30);
		set_led(0.0);
		delay(30);
	}
}

void setup(){
	//start serial connection
	printf_begin();
	Serial.begin(9600);

	pinMode(LED_DIM_PIN, OUTPUT);

	radio.begin();
	radio.powerDown();
	radio.setRetries(15, 15);
	radio.setAutoAck(false);
	radio.setChannel(95);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.openReadingPipe(1, address);
	radio.startListening();

	radio.printDetails();

	set_led(1.0);
}

void loop(){

	while (radio.available()) {
		Serial.println("Radio available");
		uint8_t payload[4];
		radio.read(payload, 4);
		printf("Received payload %d\n", payload[0], payload[1], payload[2], payload[3]);
		set_led(payload[0]/100.0);
	}

}



