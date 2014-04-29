#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int BUZZER_IN_PIN = 4; // P1
const int ALARM_BUTTON = 17; // P3
const int BUZZER_OUT_PIN = 3; // P3 analog 0
const int CE_PIN = 8;
const int CSN_PIN = 7;

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
static int doing_sunrise = 0;
static int sound_alarm = 0;

static int got_buzzer_intr;
static unsigned long buzzer_tstamps[2];
static unsigned long sound_alarm_start;
static unsigned long sunrise_start_time;
static unsigned long sunrise_stop_at;

#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))
#define ANALOG_RESOLUTION 8
#define ANALOG_MAX ((1 << ANALOG_RESOLUTION) - 1)

#define FAST_SUNRISE 0

RF24 radio(CE_PIN, CSN_PIN);

// see https://svn.kapsi.fi/jpa/led-controller/sw/src/led_task.c
const struct sequence sunrise_sequence[] = { 
		{ 0,         0,    0,    0.01},
		{ 300,       0,    0,    0.02}, // Dark blue
		{ 900,  0.05,    0.02,    0.05},
		{ 1500,  0.10,    0.02,    0.05}, // Sun begins to rise
		{ 1800,  0.20,   0.10,    0.05},
		{ 2100,   0.50,   0.12,    0.05}, // Yellowish
		{ 2400,   0.60,   0.4,   0.2}, // White
		{ 3600,   1.00,   0.65,   0.45}, // Bright white
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

	analogWrite(R, r * ANALOG_MAX);
	analogWrite(G, g * ANALOG_MAX);
	analogWrite(B, b * ANALOG_MAX);
	printf("Set LEDs to %d %d %d\n", (int)(r*100), (int)(g*100), (int)(b*100));
}

void buzzer_interrupt(void)
{	
	buzzer_tstamps[PCintPort::pinState] = millis();
	if (PCintPort::pinState == HIGH)
		got_buzzer_intr = 1;
}

/* Buzzer detect: 63 ms of silence, then 812 ms of silence +/- 2ms, then again */
int buzzer_handle_silence_time(int time)
{
	static int step = 0;
	int buzzer_wait_for = 1000;

	// Ignore tiny delays: this isn't a silence!
	if (time < 5) {
		return 0;
	}	

	// If in an alarm right now, ignore
	if (doing_sunrise) {
		return 0;
	}

	// Compute step expectation
	switch(step) {
		case 0:
		case 2:
			buzzer_wait_for = 63;
			break;
		case 1:
		case 3:
			buzzer_wait_for = 812;
			break;
	}

	printf("Waiting for %d got %d\n", buzzer_wait_for, time);

	if (abs(time - buzzer_wait_for) > 2) {
		// Not the expected delay - reset sequence
		step = 0;
		return 0;
	} 

	step++;
	Serial.println(step);
	
	if (step == 4) {
		// Step 3 completed successfully, alarm detection is TRUE
		step = 0;
		Serial.println("ALARM NOW");
		return 1;
	}

	return 0;
}

// Handle alarm situation
void alarm_now(void)
{
	// Take note that an alarm happened
	start_sunrise();

	// Shut down the alarm on the clock
	digitalWrite(ALARM_BUTTON, LOW);
	delay(100);
	digitalWrite(ALARM_BUTTON, HIGH);
	
}

void ring_buzzer()
{
	int time = millis();
	int silence = (time >> 7) & 0x01;
	int val;

	if (silence) {
		val = 0;
	} else {
		val = 127;
	}

	analogWrite(BUZZER_OUT_PIN, val);
	
	if ((time - sound_alarm_start) >> 10 > 30) {
		sound_alarm = 0;
		analogWrite(BUZZER_OUT_PIN, 0);
	}
}

void start_sunrise()
{
	doing_sunrise = 1;
	sunrise_start_time = millis();
}

void stop_sunrise()
{
	doing_sunrise = 0;
	sunrise_stop_at = millis() + 4 * 60 * 1000L;
	printf("Stopping sunrise sequence, shutting down at %ld, now is %ld\n", sunrise_stop_at, millis());
}

void sunrise()
{
	const struct sequence *s = NULL;
	int cur_seq;
	float delay_in_sunrise = (millis() - sunrise_start_time) / 1000.0;
#if FAST_SUNRISE
	delay_in_sunrise *= 60;
#endif
	float pct;
	float delta_r, delta_g, delta_b;


	// Compute current sequence number
	for (cur_seq = ARRAY_SZ(sunrise_sequence) - 1; cur_seq >= 0; cur_seq--) {
		if (delay_in_sunrise >= sunrise_sequence[cur_seq].time) {
			s = &sunrise_sequence[cur_seq];
			break;
		}
	}

	// Halt when reaching the last sequence
	if (cur_seq == sizeof(sunrise_sequence)/sizeof(sunrise_sequence[0]) - 1) {
		set_led(s->r, s->g, s->b);
		stop_sunrise();
		return;
	}

	pct = (delay_in_sunrise - s->time) / ((s+1)->time - s->time);
//	printf("Pct %d\n", (int)(pct * 1000));
	delta_r = (s+1)->r - s->r;
	delta_g = (s+1)->g - s->g;
	delta_b = (s+1)->b - s->b;

	set_led(s->r + delta_r * pct, s->g + delta_g * pct, s->b + delta_b * pct);
}

void strobe()
{
	int i = 1000;

	while (i--) {
		set_led(1.0, 1.0, 1.0);
		delay(30);
		set_led(0.0, 0.0, 0.0);
		delay(30);
	}
}

void setup(){
	//start serial connection
	printf_begin();
	Serial.begin(9600);
	printf("0\n");
	pinMode(BUZZER_IN_PIN, INPUT);
	pinMode(ALARM_BUTTON, OUTPUT); 
	pinMode(BUZZER_OUT_PIN, OUTPUT);
	PCintPort::attachInterrupt(BUZZER_IN_PIN, &buzzer_interrupt, CHANGE);
	digitalWrite(ALARM_BUTTON, HIGH);
	printf("1\n");
	radio.begin();
	radio.setRetries(15, 15);
	printf("1\n");
	radio.setAutoAck(false);
	printf("1\n");
	radio.setChannel(95);
	printf("1\n");
	radio.setPayloadSize(sizeof(unsigned long));
	printf("1\n");
	radio.setPALevel(RF24_PA_MAX);
	printf("1\n");
	radio.setDataRate(RF24_250KBPS);
	printf("3\n");

	printf("a\n");
	radio.printDetails();
	printf("b\n");

	bitSet(TCCR1B, WGM12);
}

void loop(){
	// If buzzer is ringing, detect if this is an alarm
	if (got_buzzer_intr) {
		long time = buzzer_tstamps[1] - buzzer_tstamps[0];

		if (buzzer_handle_silence_time(time)) {
			alarm_now();
		}

		got_buzzer_intr = 0;
	}

	// Should we do the sunrise sequence?
	if (doing_sunrise) {
		sunrise();
	} else if (sunrise_stop_at && millis() > sunrise_stop_at) {
		set_led(0, 0, 0);
		sunrise_stop_at = 0;
	}

	// Should we emit a sound?
	if (sound_alarm) {
		ring_buzzer();
	}

}



