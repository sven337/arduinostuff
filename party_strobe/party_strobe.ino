#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int ON_OFF_PIN = 4; // P1 D
const int BUTTON_PIN = 3; // P1 IRQ
const int SPEED_PIN = A0; // P1 A

const int R = 6;
const int G = 9;
const int B = 5;

struct sequence {
	int time;
	float r;
	float g;
	float b;
};

// "Worklist"
enum sequence_type {
	NONE = 0,
	STROBE = 1,
	SUNRISE = 2,
	SUNSET = 3,
	LAST_SEQ,
} doing_sequence;

static unsigned long sequence_start_time;

static float sequence_speed_factor = 1.0;
static long last_speed_factor_at;

static int state_off = 0;

#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))
#define ANALOG_RESOLUTION 8
#define ANALOG_MAX ((1 << ANALOG_RESOLUTION) - 1)

static uint8_t led_current_r;
static uint8_t led_current_g;
static uint8_t led_current_b;

//RF24 radio(CE_PIN, CSN_PIN);
//const uint64_t address_pi = 0xF0F0F0F0F1LL;

// see https://svn.kapsi.fi/jpa/led-controller/sw/src/led_task.c
const struct sequence sunrise_sequence[] = { 
		{ 0,         0,    0,    0.01},
		{ 3,       0,    0,    0.02}, // Dark blue
		{ 9,  0.05,    0.02,    0.05},
		{ 15,  0.10,    0.02,    0.05}, // Sun begins to rise
		{ 18,  0.20,   0.10,    0.05},
		{ 21,   0.50,   0.12,    0.05}, // Yellowish
		{ 24,   0.60,   0.4,   0.2}, // White
		{ 36,   1.00,   0.65,   0.45}, // Bright white
};

const struct sequence sunset_sequence[] = { 
		{ 0,   1.00,   0.65,   0.45}, // Bright white
		{ 2,   0.60,   0.4,   0.2}, // White
		{ 5,   0.50,   0.12,    0.05}, // Yellowish
		{ 8,  0.20,   0.10,    0.05},
		{ 9,  0.10,    0.02,    0.05}, // Sun begins to rise
		{ 10,  0.05,    0.02,    0.05},
		{ 12,       0,    0,    0.02}, // Dark blue
		{ 18,         0,    0,    0.01},
};

float bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (exp(0.6931471805599453 * in) - 1.0);
}

void set_led(float r, float g, float b)
{
	r = bri(r);
	g = bri(g);
	b = bri(b);

	r *= ANALOG_MAX;
	g *= ANALOG_MAX;
	b *= ANALOG_MAX;

	led_current_r = r;
	led_current_g = g;
	led_current_b = b;

	analogWrite(R, r);
	analogWrite(G, g);
	analogWrite(B, b);
}

void radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
/*	uint8_t payload[4] = { p0, p1, p2, p3 };
	radio.stopListening();
	delay(10);
	radio.powerUp();
	bool ok = radio.write(payload, 4);
	if (ok) 
		Serial.println("send ok");
	else
		Serial.println("send KO");
	delay(1);
	radio.startListening();*/
}

void start_sequence(int which)
{
//	printf("Starting sequence %d\n", which);
	doing_sequence = (enum sequence_type) which;
	sequence_start_time = millis();
	radio_send('S', which, 0, 0);
}

void stop_sequence()
{
	doing_sequence = NONE;
	set_led(0, 0, 0);
//	printf("Stopping sequence\n");
}

float get_delay_in_sequence()
{
	float delay_in_sequence = (millis() - sequence_start_time) / 1000.0;
	delay_in_sequence *= sequence_speed_factor;
	return delay_in_sequence;
}

void led_sequence(const struct sequence *seq, int seq_size)
{
	const struct sequence *s = NULL;
	int cur_seq;
	float delay_in_sequence = get_delay_in_sequence();
	float pct;
	float delta_r, delta_g, delta_b;

	// Compute current sequence number
	for (cur_seq = seq_size - 1; cur_seq >= 0; cur_seq--) {
		if (delay_in_sequence >= seq[cur_seq].time) {
			s = &seq[cur_seq];
			break;
		}
	}

	// Halt when reaching the last sequence
	if (cur_seq == seq_size - 1) {
		set_led(s->r, s->g, s->b);
		stop_sequence();
		return;
	}

	pct = (delay_in_sequence - s->time) / ((s+1)->time - s->time);
	delta_r = (s+1)->r - s->r;
	delta_g = (s+1)->g - s->g;
	delta_b = (s+1)->b - s->b;
	//printf("Pct %d, delta_r %d\n", (int)(pct * 1000), (int)(delta_r * 1000));

	set_led(s->r + delta_r * pct, s->g + delta_g * pct, s->b + delta_b * pct);
}

void on_off_interrupt(void)
{
	state_off = PCintPort::pinState;
}

void button_interrupt(void)
{
//	Serial.println("Button interrupt");
	if (doing_sequence == LAST_SEQ - 1)
		stop_sequence();
	else start_sequence(doing_sequence + 1);
}

void strobe()
{
	static float next = 0.0;
	static long next_at = 0;

	long now = millis();
	int wait = constrain(13 + 15 / sequence_speed_factor, 1, 500);

	float freq = 1 + sequence_speed_factor * 30;
	wait = 1000.0 / freq;
	printf("Wait = %d\n", wait);

	if (now > next_at) {
		set_led(next, next, next);
		next_at = now + wait;
		printf("Wait = %d\n", wait);
		next = 1.0 - next;
	} else {
		if (next_at - now > wait)
			next_at = now + wait;
	}
}

void setup(){
	//start serial connection
	printf_begin();
	Serial.begin(57600);
	pinMode(ON_OFF_PIN, INPUT);
	digitalWrite(ON_OFF_PIN, HIGH);
	pinMode(BUTTON_PIN, INPUT);
	digitalWrite(BUTTON_PIN, HIGH);

	pinMode(SPEED_PIN, INPUT);
    digitalWrite(SPEED_PIN, HIGH);

	printf("setup\n");
	PCintPort::attachInterrupt(ON_OFF_PIN, &on_off_interrupt, CHANGE);
	PCintPort::attachInterrupt(BUTTON_PIN, &button_interrupt, FALLING);
	/*radio.begin();
	radio.setRetries(15, 15);
	radio.setAutoAck(true);
	radio.setChannel(95);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.openReadingPipe(1, address_pi);
	radio.openWritingPipe(address_pi);
	radio.startListening();

	radio.printDetails();*/

	// fix PWM (LED flicker)
	bitSet(TCCR1B, WGM12);

	state_off = digitalRead(ON_OFF_PIN);
}

void loop(){

	if (last_speed_factor_at - millis() > 500) {
	   last_speed_factor_at = millis();
   	   float val = analogRead(SPEED_PIN) / 1024.0;
	   val = - (33000.0/36000.0) * val / (val - 1);
	   val = constrain(val, 0.0, 1.0);
//	   Serial.println(val);
	   sequence_speed_factor = val;

	   printf("Sequence %d\n", doing_sequence);
	}	   

	if (state_off) {
		set_led(0, 0, 0);
		return;
	}

	// Should we do the sunrise sequence?
	switch (doing_sequence) {
		case SUNRISE:
			led_sequence(sunrise_sequence, ARRAY_SZ(sunrise_sequence));
			break;
		case SUNSET:
			led_sequence(sunset_sequence, ARRAY_SZ(sunset_sequence));
			break;
		case STROBE:
			strobe();
			break;
		case NONE:
			//fall through to save 2 bytes of program memory :)
			set_led(0, 0, 0);
		default:
			doing_sequence = NONE;
	}

/*
	while (radio.available()) {
		Serial.println("Radio available");
		uint8_t payload[4];
		radio.read(payload, 4);
		// Sequence
		if (payload[0] == 'S') {
			start_sequence(payload[1]);
		} else if (payload[0] == 'F') {
			fast_sequence = !fast_sequence;
			radio_send('F', fast_sequence, 0, 0);
		} else if (payload[0] == 'L') {
			radio_send('L', led_current_r, led_current_g, led_current_b);
			radio_send('S', doing_sequence, (int)get_delay_in_sequence() & 0xFF, ((int)get_delay_in_sequence() >> 8) & 0xFF);
		} else if (payload[0] == 'V') {
			set_led(payload[0]/255.0, payload[1]/255.0, payload[2]/255.0);
			radio_send('L', led_current_r, led_current_g, led_current_b);
		}
	}*/
}



