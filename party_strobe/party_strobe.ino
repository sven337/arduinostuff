#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

const int ON_OFF_PIN = 7;
const int BUTTON_PIN = 8;
const int SPEED_PIN = A0;
const int SOUND_PIN = A3;

const int R = 6;
const int G = 5;
const int B = 9;

struct sequence {
	int time;
	float r;
	float g;
	float b;
};

static float sequence_speed_factor = 1.0;
static unsigned long last_speed_factor_at;

static bool strobe_off;
/* Ramps */
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

enum mode {
	OFF = 0, 
	MUSIC,
	RAMP,
	STROBE,
} current_mode; 

enum ramp_name {
    FADE7 = 1,
    FADE7_END = 7,
};

static uint8_t doing_ramp;

struct ramp {
    long duration;
    float r;
    float g;
    float b;
    uint8_t next_ramp;
};

const int dur = 5000; 

const struct ramp LED_ramps[] = {
                  { 1,     0,      0,      0, 0 }, //OFF
        [FADE7] = { dur,     0.0,   0.0,     1.0, FADE7+1 }, //FADE7
                { dur,   0.0,   1.0,     1.0, FADE7+2 },
                { dur,     0.0,   1.0,       0, FADE7+3 },
                { dur,   1.0,   1.0,     0.0, FADE7+4 },
                { dur,   1.0,      0,      0, FADE7+5 },
                { dur,   1.0,   0.0,     1.0, FADE7+6 },
        [FADE7_END] = { dur,   0.7,   0.7,     0.7, FADE7 }, //FADE7_END
};


void on_off_interrupt(void)
{
	// Potentiometer on/off enables/disables strobe from music mode
	strobe_off = PCintPort::pinState;
	printf("strobe_off: %d\n", strobe_off);
}

void button_interrupt(void)
{
	if (current_mode == STROBE) {
		current_mode = OFF;
	} else {
		current_mode = (enum mode)(current_mode + 1);
	}

	printf("current mode is %d\n", current_mode);
}

uint8_t bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
//	return (uint8_t)(255.0 * (exp(0.6931471805599453 / 255.0 * in) - 1.0));
	return (uint8_t)(255.0 * in);
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

void start_ramp(int which)
{
	doing_ramp = which;
	const struct ramp *r = &LED_ramps[doing_ramp];

	ramp_start_time = millis();
	ramp_last_step = ramp_start_time;

	// Compute per-millisecond deltas
	delta_r = (I2F((int32_t)(255.0 * r->r)) - led_r) / r->duration;
	delta_g = (I2F((int32_t)(255.0 * r->g)) - led_g) / r->duration;
	delta_b = (I2F((int32_t)(255.0 * r->b)) - led_b) / r->duration;

	printf("Start ramp %d deltas r %ld g %ld b %ld\n", doing_ramp, delta_r, delta_g, delta_b);
	current_mode = RAMP;
	if (which == OFF) {
		// OFF ramp isn't used: set current_mode to OFF
		current_mode = OFF;
	}
}

void led_ramp(void)
{
	const struct ramp *r = &LED_ramps[doing_ramp];
	const unsigned long now = millis();
	unsigned long delay_in_ramp = now - ramp_start_time;
	if (delay_in_ramp > r->duration) {
		// Ramp is done, move on
		if (r->next_ramp != doing_ramp) {
			set_led(I2F((uint32_t)(255.0 * r->r)), I2F((uint32_t)(255.0 * r->g)), I2F((uint32_t)(255.0 * r->b)));
			start_ramp(r->next_ramp);
		}
		return;
	}

	unsigned long this_step_duration = now - ramp_last_step;
	ramp_last_step = now;

	set_led(led_r + this_step_duration * delta_r, led_g + this_step_duration * delta_g, led_b + this_step_duration * delta_b);
}

void strobe()
{
	static int last_wait = 0;
	static fixedpoint next = 0;
	static long next_at = 0;

	long now = millis();
	int wait = constrain(13 + 15 / sequence_speed_factor, 1, 500);

	float freq = 1 + sequence_speed_factor * 30;
	wait = 1000.0 / freq;
	if (wait != last_wait) {
		printf("Wait = %d\n", wait);
		last_wait = wait;
	}

	if (now > next_at) {
		set_led(next, next, next);
		next_at = now + wait;
		next = I2F((uint32_t)255) - next;
		printf("next = %d\n", next);
	} else {
		if (next_at - now > wait)
			next_at = now + wait;
	}
}

void music()
{
	// FFT and set leds accordingly
}

void setup()
{
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

	// fix PWM (LED flicker)
	bitSet(TCCR1B, WGM12);

	strobe_off = digitalRead(ON_OFF_PIN);

	current_mode = RAMP;
	start_ramp(FADE7);
}

void loop()
{

	if (millis() - last_speed_factor_at > 500) {
	   last_speed_factor_at = millis();
   	   float val = analogRead(SPEED_PIN) / 1024.0;
	   val = - (33000.0/36000.0) * val / (val - 1);
	   val = constrain(val, 0.0, 1.0);
	   sequence_speed_factor = val;
	}	   


	switch (current_mode) {
		case OFF:
			set_led(0, 0, 0);
			break;
		case RAMP:
			// XXX when should we start_ramp()?
			led_ramp();
			break;
		case STROBE:
			if (!strobe_off) {
				strobe();
				break;
			}
			// Fall through
		case MUSIC:
			music();
			break;
	}
}



