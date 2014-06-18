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
enum sequence_type {
	NONE = 0,
	SUNRISE = 1,
	SUNSET = 2,
	STROBE = 3,
} doing_sequence;
static int sound_alarm = 0;

static int got_buzzer_intr;
static unsigned long buzzer_tstamps[2];
static unsigned long sound_alarm_start;
static unsigned long sequence_start_time;
static unsigned long lights_out_at;

#define ARRAY_SZ(X) (sizeof(X)/sizeof(X[0]))
#define ANALOG_RESOLUTION 8
#define ANALOG_MAX ((1 << ANALOG_RESOLUTION) - 1)

static int fast_sequence = 0;
	
static uint8_t led_current_r;
static uint8_t led_current_g;
static uint8_t led_current_b;

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address_pi = 0xF0F0F0F0F1LL;

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

const struct sequence sunset_sequence[] = { 
		{ 0,   1.00,   0.65,   0.45}, // Bright white
		{ 150,   0.60,   0.4,   0.2}, // White
		{ 450,   0.50,   0.12,    0.05}, // Yellowish
		{ 750,  0.20,   0.10,    0.05},
		{ 900,  0.10,    0.02,    0.05}, // Sun begins to rise
		{ 1050,  0.05,    0.02,    0.05},
		{ 1200,       0,    0,    0.02}, // Dark blue
		{ 1800,         0,    0,    0.01},
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
	if (doing_sequence) {
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
	start_sequence(SUNRISE);

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
	delay(1);
	radio.startListening();
}

void start_sequence(int which)
{
	doing_sequence = (enum sequence_type) which;
	sequence_start_time = millis();
	radio_send('S', which, 0, 0);
}

void stop_sequence()
{
	doing_sequence = NONE;
	lights_out_at = millis() + 4 * 60 * 1000L;
	printf("Stopping sequence, shutting down lights at %ld, now is %ld\n", lights_out_at, millis());
}

float get_delay_in_sequence()
{
	float delay_in_sequence = (millis() - sequence_start_time) / 1000.0;
	if (fast_sequence) {
		delay_in_sequence *= 60;
	}
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
		printf("Pct %d, delta_r %d\n", (int)(pct * 1000), (int)(delta_r * 1000));

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
	pinMode(BUZZER_IN_PIN, INPUT);
	pinMode(ALARM_BUTTON, OUTPUT); 
	pinMode(BUZZER_OUT_PIN, OUTPUT);
	PCintPort::attachInterrupt(BUZZER_IN_PIN, &buzzer_interrupt, CHANGE);
	digitalWrite(ALARM_BUTTON, HIGH);
	radio.begin();
	radio.powerDown();
	radio.setRetries(15, 15);
	radio.setChannel(95);
	radio.setCRCLength(RF24_CRC_16);
	radio.setPayloadSize(sizeof(unsigned long));
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
 	radio.setAutoAck(true);
	radio.openReadingPipe(1, address_pi);
	radio.openWritingPipe(address_pi);
	radio.startListening();

	radio.printDetails();

	// fix PWM (LED flicker)
	bitSet(TCCR1B, WGM12);

	doing_sequence = NONE;
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
			if (lights_out_at && millis() > lights_out_at) {
				set_led(0, 0, 0);
				lights_out_at = 0;
			}
			//fall through to save 2 bytes of program memory :)
		default:
			doing_sequence = NONE;
	}

	// Should we emit a sound?... not with my broken buzzer
	if (sound_alarm) {
		ring_buzzer();
	}

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
	}
}



