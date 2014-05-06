#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int LED_DIM_PIN = 3; // P3 analog 0
const int CE_PIN = 8;
const int CSN_PIN = 9;
const int LDR_PIN = A0;

#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xF0F0F0F0F2LL;

int target_light_level = 160;
float light_duty_cycle = 1.0;

float bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (exp(0.6931471805599453 * in) - 1.0);
}

void set_led(float val)
{
//	val = bri(val);
	analogWrite(LED_DIM_PIN, val * 255);
	printf("Set LED to %d%%\n", (int)(val*100));
}

int get_light_level()
{
	return analogRead(LDR_PIN);
}

int increase_light_output()
{
	light_duty_cycle += 0.01;
	if (light_duty_cycle > 1.0)
		light_duty_cycle = 1.0;
	set_led(light_duty_cycle);
}

int decrease_light_output()
{
	light_duty_cycle -= 0.01;
	if (light_duty_cycle < 0.0);
		light_duty_cycle = 0.0;
	set_led(light_duty_cycle);
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

	set_led(light_duty_cycle);

	// Light level sensor
	pinMode(LDR_PIN, INPUT);
	digitalWrite(LDR_PIN, HIGH);
}

void loop(){

	int light_level = get_light_level();
	if (light_level > (float)target_light_level * 1.1) {
		printf("Light at %d, high target %d, increasing duty cycle.\n", light_level, (int)(1.1 * (float)target_light_level));
		increase_light_output();
	} else if (light_level < (float)target_light_level * 0.9) {
		decrease_light_output();
		printf("Light at %d, low target %d, decreasing duty cycle.\n", light_level, (int)(0.9 * (float)target_light_level));
	}

	while (radio.available()) {
		Serial.println("Radio available");
		uint8_t payload[4];
		radio.read(payload, 4);
		printf("Received payload %d\n", payload[0], payload[1], payload[2], payload[3]);
		set_led(payload[0]/100.0);
	}

}



