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
	FADEOUT_END = 10,
};

static uint8_t doing_ramp;

const struct ramp LED_ramps[] = {
		{ 1, 	   0, 	   0, 	   0, 0 }, //OFF
		{ 1,	0.02,	0.01,	   0, 2 }, //RISE_START
		{ 200,  0.15,	   0,	0.08, RISE_END	},
		{ 500,	 0.1,	0.05,	0.02, 4	},
		{ 500,	 0.2,	 0.1,	0.04, 5	},
		{ 2500,	 0.5,	0.25,	0.04, 6	},
		{ 1500,	 0.7,	0.35,	0.04, RISE_END },
		{ 1000,	   1,   0.35,	0.08, RISE_END }, // RISE_END	
		{ 1000, 0.15,	   0,   0.13, 9 }, // FADEOUT_START
		{ 1000, 0.15,	   0,   0.08, FADEOUT_END },
		{ 4000,    0,	   0,      0, FADEOUT_END }, // FADEOUT_END
};

uint8_t bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (uint8_t)(255.0 * (exp(0.6931471805599453 / 255.0 * in) - 1.0));
}

void set_led(long r, long g, long b) //fixedpoint!
{
	led_r = r;
	led_g = g;
	led_b = b;

	uint8_t my_r, my_g, my_b;
#define BRI_CORRECT(X) (bri(F2I(X)) & 0xFF)
	my_r = BRI_CORRECT(r);
	my_g = BRI_CORRECT(g);
	my_b = BRI_CORRECT(b);
	printf("set led to %d %d %d\n", my_r, my_g, my_b);
	analogWrite(R, my_r);
	analogWrite(G, my_g);
	analogWrite(B, my_b);
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



