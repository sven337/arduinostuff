#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 

const int PIR_PIN = 4; // P1 D
const int DIAG_LED_PIN = 7; // P3 D

const int R = 6;
const int G = 9;
const int B = 5;

static int got_pir_intr = 0;
static unsigned long presence_detected_at;

struct ramp {
	int time;
	float r;
	float g;
	float b;
	int next_ramp;
};

typedef signed long fixedpoint;

#define I2F(I) (I << 20)
#define F2I(F) (F >> 20)

static fixedpoint led_r = 0;
static fixedpoint led_g = 0;
static fixedpoint led_b = 0;

static fixedpoint delta_r;
static fixedpoint delta_g;
static fixedpoint delta_b;

static unsigned long ramp_start_time;
static unsigned long ramp_last_step;

enum ramp_name {
	OFF,
	RISE_START,
	RISE_END = 7,
	FADEOUT_START,
	FADEOUT_END,
};

static uint8_t doing_ramp
;
const struct ramp LED_ramps[] = {
		{ 1, 	   0, 	   0, 	   0, 0 }, //OFF
		{ 500,	   0,	   0,	0.02, 2 }, //RISE_START
		{ 500, 	0.04,	0.02,	0.04, 3	},
		{ 500,	 0.1,	0.02,	0.04, 4	},
		{ 500,	 0.2,	 0.1,	0.04, 5	},
		{ 2000,	 0.5,	0.15,	0.05, 6	},
		{ 1000,	0.55,	0.35,	0.15, 7	},
		{ 1000,	 1.0,   0.65,	0.45, RISE_END }, // RISE_END	
		{ 2000, 0.15,	   0,   0.15, FADEOUT_END }, // FADEOUT_START
		{ 4000,    0,	   0,      0, FADEOUT_END }, // FADEOUT_END
};

uint8_t bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return 255.0 * (exp(0.6931471805599453 * in) - 1.0);
}

void set_led(long r, long g, long b) //fixedpoint!
{
	led_r = r;
	led_g = g;
	led_b = b;

	printf("set led %lu %lu %lu\n", F2I(r), F2I(g), F2I(b));
/*	analogWrite(R, bri(r>>23));
	analogWrite(G, bri(g>>23));
	analogWrite(B, bri(b>>23));*/
	analogWrite(R, F2I(r) & 0xFF);
	analogWrite(G, F2I(g) & 0xFF);
	analogWrite(B, F2I(b) & 0xFF);
}

void pir_interrupt(void)
{
	got_pir_intr = 1;
}

void start_ramp(int which)
{
	doing_ramp = which;
	const struct ramp *r = &LED_ramps[doing_ramp];

	ramp_start_time = millis();
	ramp_last_step = ramp_start_time;

	// Compute per-millisecond deltas
	delta_r = (I2F((int32_t)(255.0 * r->r)) - led_r) / r->time;
	delta_g = (I2F((int32_t)(255.0 * r->g)) - led_g) / r->time;
	delta_b = (I2F((int32_t)(255.0 * r->b)) - led_b) / r->time;

	printf("Starting ramp %d deltas are r %ld g %ld b %ld\n", doing_ramp, delta_r, delta_g, delta_b);
}

void led_ramp(void)
{
	const struct ramp *r = &LED_ramps[doing_ramp];
	const unsigned long now = millis();
	unsigned long delay_in_ramp = now - ramp_start_time;
	if (delay_in_ramp > r->time) {
		// Ramp is done, move on
		if (r->next_ramp != doing_ramp) {
			set_led(I2F((uint32_t)(255.0 * r->r)), I2F((uint32_t)(255.0 * r->g)), I2F((uint32_t)(255.0 * r->b)));
			start_ramp(r->next_ramp);
		} 
		return;
	}

	unsigned long this_step_duration = now - ramp_last_step;
	ramp_last_step = now;

//	printf("step duration %ld\n", this_step_duration);
	set_led(led_r + this_step_duration * delta_r, led_g + this_step_duration * delta_g, led_b + this_step_duration * delta_b);
}

void setup(){
	//start serial connection
	printf_begin();
	Serial.begin(57600);
	pinMode(PIR_PIN, INPUT);
	pinMode(DIAG_LED_PIN, OUTPUT);
	digitalWrite(DIAG_LED_PIN, 0);
	PCintPort::attachInterrupt(PIR_PIN, &pir_interrupt, CHANGE);

	// Trigger false interrupt reading on PIR pin to not miss its
	// current state
	got_pir_intr = 1;

	// fix PWM (LED flicker)
	bitSet(TCCR1B, WGM12);

	start_ramp(OFF);
	printf("Ready!\n");
}

void loop(){
	if (got_pir_intr) {
		if (digitalRead(PIR_PIN) && !presence_detected_at) {
			// Detected someone
			presence_detected_at = millis();
			printf("Human here\n");
			start_ramp(RISE_START);
			digitalWrite(DIAG_LED_PIN, 1);
		} else {
			printf("PIR doesn't see us any longer.");
		}
		got_pir_intr = 0;
	}
	
	if (presence_detected_at && (millis() - presence_detected_at > 15000)) {
		// Timeout on presence
		presence_detected_at = 0;
		digitalWrite(DIAG_LED_PIN, 0);
		start_ramp(FADEOUT_START);
	}

	led_ramp();
}



