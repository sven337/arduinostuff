#include <SPI.h>
#include <OneWire.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"


const int LED_DIM_PIN = 3; // [P3] "IRQ"
const int CE_PIN = 8;
const int CSN_PIN = 9;
//const int LDR_PIN = A0; // P1 A
const int THERM_PIN = A1; // P2 A
const int DS18B20_PIN = 7; // P4 D
#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address = 0xF0F0F0F0F2LL;

OneWire ds(DS18B20_PIN);

int lamp_off = 0;
uint8_t led_power = 200;
uint8_t old_led_power = 0;
bool thermal_override = 0;

unsigned long next_temperature_check_at;
unsigned long next_lightlevel_send_at;
unsigned long fade_to_black_start_date;
unsigned long next_ambient_temp_report_at;

void set_led(uint8_t val)
{
	analogWrite(LED_DIM_PIN, val);
	printf("Set LED to %d%%\n", val*100/255);
}

void radio_send_led_power(uint8_t event_type)
{
	if (led_power == 0 || led_power == 255 || millis() > next_lightlevel_send_at) {
		radio_send('D', event_type, (led_power * 100 / 255), 0);
		next_lightlevel_send_at = millis() + 10000;
	}
}

void strobe()
{
	int i = 5;

	while (i--) {
		set_led(0);
		delay(30);
		set_led(led_power);
		delay(30);
	}
}

void stop_lamp()
{
	lamp_off = 1;
	printf("Lamp is now off.\n");
	set_led(0);
	radio_send_led_power();
}

void fadeout()
{
	if (millis() >= fade_to_black_start_date + 15000) {
		stop_lamp();
		fade_to_black_start_date = 0;
		return;
	}
	
	unsigned long remaining = fade_to_black_start_date + 15000 - millis();
	if (remaining < 10000) {
		set_led(led_power * remaining / 10000);
	}
}

int get_temperature()
{
	// Integrated pullup has R1 = 20k
	// We have Uth = Rth * E / (R1 + Rth)
    // let V = Uth * 1023 / E value read by the ADC
    // <=> V = 1023 * Rth / (R1 + Rth)
	// solve in Rth :
    // Rth 	=  R1 * V / (1023 - V)

	// Once we have Rth, use beta parameter equation for NTC:
	// T = Beta / ln (Rth / R0 * exp(-Beta/T0))

	unsigned int adc = analogRead(THERM_PIN);
	const unsigned long int R1 = 21900;
	const float Beta = 4400.0;
	const float Beta_R0 = 100000.0;
	const float Beta_T0 = 25.0+273.0;
	float Rth = R1 * adc / (float)(1023 - adc);

	float T = Beta / log(Rth / (Beta_R0 * exp(-Beta / Beta_T0))) - 273.0;
//	printf("adc is %d, R1*adc is %ld, Rth is %lu, temp is %u\n", adc, R1*adc, (long unsigned int)Rth, (int)(T * 100.0));
	
	return (int)(T * 100.0);
}

int get_ambient_temperature()
{
	uint8_t addr[8];
	uint8_t data[12];
	int i = 4;
	while(!ds.search(addr) && i--) {
		Serial.println("No more addresses.");
		ds.reset_search();
		delay(250);
	}
    if (!i) {
		return 0;
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
	printf("ambient Temperature is %d\n", (int)(100.0*raw/16.0));

	return (int)(((float)raw/16.0) * 100.0);
}

void radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
	uint8_t payload[4] = { p0, p1, p2, p3 };
	radio.stopListening();
	bool ok = radio.write(payload, 4);
	if (ok) 
		Serial.println("send ok");
	else
		Serial.println("send KO");
	radio.startListening();
}

void radio_send_temperature(uint8_t type, int temperature)
{
	radio_send('T', type, temperature & 0xFF, (temperature >> 8) & 0xFF);
}

void radio_send_led_power()
{
	if (lamp_off) {
		radio_send('R', 'O', 'F', 'F');
	} else {
		radio_send('R', '1', led_power & 0xFF, (led_power >> 8) & 0xFF);
	}
}

void radio_send_ambient(uint16_t deg)
{
	radio_send('A', 'N', (deg >> 8) & 0xFF, deg & 0xFF);
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
	radio.startListening();

	radio.printDetails();

	// Start LED 
	pinMode(LED_DIM_PIN, OUTPUT);
	set_led(led_power);

	// Thermistor
	pinMode(THERM_PIN, INPUT);
//	digitalWrite(THERM_PIN, LOW);
	printf("thermistor Temperature is %d\n", get_temperature());

}

uint8_t percent_to_led_power(uint8_t pct)
{
    uint16_t out = pct * 255 / 100;
    return (uint8_t)out;
}

void loop(){

	if (fade_to_black_start_date) {
		fadeout();
	}

	if (millis() > next_temperature_check_at) {
		int temp = get_temperature();
		next_temperature_check_at = millis() + 30000;
		if (temp > 8000) {
			printf("Thermal emergency, temp %d.", temp);
			radio_send_temperature('E', temp);
			thermal_override = 1;
            set_led(0);
		} else if (temp > 6000) {// 60 C 
			printf("Thermal alarm, temp %d.", temp);
			radio_send_temperature('A', temp);
			thermal_override = 1;
            set_led(0);
			next_temperature_check_at = millis() + 5000;
		} else if (thermal_override && temp < 5500) {
			radio_send_temperature('0', temp);
			printf("End thermal alarm.");
			thermal_override = 0;
		}
	}

	if (millis() > next_ambient_temp_report_at) {
		int deg = get_ambient_temperature();
		printf("Ambient temp %d\n", deg);
		radio_send_ambient(deg);
		next_ambient_temp_report_at = millis() + 15L * 60L * 1000L;
	}

	while (radio.available()) {
		uint8_t payload[4];
		radio.read(payload, 4);
		switch (payload[0]) {
			case 'L':
				if (payload[1] == 0) {
					stop_lamp();
				} else {
					lamp_off = 0;
                    set_led(percent_to_led_power(payload[1]));
					radio_send_led_power();
				}
				break;
			case 'F':
				printf("Fading out to black...\n");
				fade_to_black_start_date = millis();
				// Notify user that we're shutting down with a brief strobe
				strobe();
				break;
		}
	}
}



