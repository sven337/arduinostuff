#include <SPI.h>
#include <OneWire.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int LED_DIM_PIN = 3; // [P3] "IRQ"
const int CE_PIN = 8;
const int CSN_PIN = 9;
const int LDR_PIN = A0; // P1 A
const int THERM_PIN = 5; // P2 D

#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))
#define DIM_MAX 1000.0

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address = 0xF0F0F0F0F2LL;

OneWire ds(THERM_PIN);

int lamp_off = 0;
int target_light_level = 90;
float light_duty_cycle = 1.0;
bool thermal_override = 0;

unsigned long next_temperature_check_at;
unsigned long next_lightlevel_check_at;

float bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (exp(0.6931471805599453 * in) - 1.0);
}

int percent_to_light_level(int pct)
{
	// light level goes from 4096 to 0 when pct goes from 0 to 100
	// however, the *useful* dimming range is not 4096 to 0, so restrict it to DIM_MAX to 0

	return constrain(DIM_MAX * (100 - pct) / 100.0, 0, 4096);
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

int change_light_output(float pct)
{
	char change = 0;
	float old_duty_cycle = light_duty_cycle;
	light_duty_cycle += pct;
	light_duty_cycle = constrain(light_duty_cycle, 0.0, 1.0);
	
	if (light_duty_cycle > old_duty_cycle) {
		change = 'I';
	} else if (light_duty_cycle < old_duty_cycle) {
		change = 'D';
	}

	if (change) {
		set_led(light_duty_cycle);
		radio_send('L', change, (uint8_t)(light_duty_cycle * 100.0), 0);
	}
}

int increase_light_output(float pct = 0.01)
{
	change_light_output(pct);
}

int decrease_light_output(float pct = 0.01)
{
	change_light_output(-pct);
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

int get_temperature()
{
	uint8_t addr[8];
	uint8_t data[12];
	uint8_t present = 0;
	uint8_t i;


	if (!ds.search(addr)) {
		printf("No 1wire address.\n");
		return 60000;
	}

	if (OneWire::crc8(addr, 7) != addr[7]) {
		printf("Addr CRC error\n");
		return 60000;
	}

	if (addr[0] != 0x28) {
		printf("Addr[0] = %c, not 0x22\n", addr[0]);
		return 60000;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44, 1);
	delay(1000);

	present = ds.reset();
	ds.select(addr);
	ds.write(0xBE);

/*	Serial.print("  Data = ");
	Serial.print(present, HEX);
	Serial.print(" ");*/
	for ( i = 0; i < 9; i++) {           // we need 9 bytes
		data[i] = ds.read();
	}

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
	float celsius = (float)raw / 16.0;

	printf("Temperature is %d\n",(int)( celsius * 100));
	ds.reset_search();

	return (int)(celsius * 100.0);
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

void setup(){
	printf_begin();
	Serial.begin(9600);

	// Radio init
	radio.begin();
	radio.powerDown();
	radio.setRetries(15, 10);
	radio.setChannel(95);
	radio.setCRCLength(RF24_CRC_16);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
 	radio.setAutoAck(true);
	radio.openWritingPipe(pipe_address);
	radio.openReadingPipe(1, pipe_address);
	radio.startListening();

	radio.printDetails();

	// Start LED at full power
	pinMode(LED_DIM_PIN, OUTPUT);
	set_led(1.0);

	// Light level sensor
	pinMode(LDR_PIN, INPUT);
	digitalWrite(LDR_PIN, HIGH);
}

void loop(){

	int light_level = get_light_level();
	if (!thermal_override && !lamp_off && millis() > next_lightlevel_check_at) {
		if (light_level > (float)target_light_level * 1.1) {
			increase_light_output();
//			printf("Light at %d, high target %d, increasing duty cycle.\n", light_level, (int)(1.1 * (float)target_light_level));
		} else if (light_level < (float)target_light_level * 0.9) {
			decrease_light_output();
//			printf("Light at %d, low target %d, decreasing duty cycle.\n", light_level, (int)(0.9 * (float)target_light_level));
		}
		next_lightlevel_check_at = millis() + 100;
	}

	if (millis() > next_temperature_check_at) {
		int temp = get_temperature();
		next_temperature_check_at = millis() + 30000;
		if (temp > 8000) {
			printf("Thermal emergency, temp %d.", temp);
			radio_send('T', 'E', temp/100, 0);
			thermal_override = 1;
			decrease_light_output(1.0);
		} else if (temp > 6000) {// 60 C 
			printf("Thermal alarm, temp %d.", temp);
			radio_send('T', 'A', temp/100, 0);
			thermal_override = 1;
			decrease_light_output(0.1);
			next_temperature_check_at = millis() + 5000;
		} else if (thermal_override && temp < 5500) {
			radio_send('T', '0', temp/100, 0);
			printf("End thermal alarm.");
			thermal_override = 0;
		}
	}

	while (radio.available()) {
		uint8_t payload[4];
		radio.read(payload, 4);
		if (payload[0] == 0) {
			lamp_off = 1;
			printf("Lamp is now off.\n");
			radio_send('R', 'O', 'F', 'F');
			set_led(0.0);
			decrease_light_output(1.0);
		} else {
			lamp_off = 0;
			target_light_level = percent_to_light_level(payload[0]);
			printf("Set target light level %d\n", target_light_level);
			radio_send('R', '1', target_light_level & 0xFF, (target_light_level >> 8) & 0xFF);
		}
	}
	
	if (Serial.available()) {
		uint8_t payload;
		payload = Serial.read();
		light_duty_cycle = payload/100.0;
		set_led(light_duty_cycle);
	}


}



