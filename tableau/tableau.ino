#include <SPI.h>
#include <PinChangeInt.h>
#include <IRremote.h>
#include "printf.h" 
#include "remote_control.h"

const int DIAG_LED_PIN = 7; // P3 D

const int R = 9;
const int G = 5;
const int B = 6;
const int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);


static unsigned long presence_detected_at;

typedef signed long fixedpoint;

#define I2F(I) (I << 20)
#define F2I(F) (F >> 20)

static fixedpoint led_r = 0;
static fixedpoint led_g = 0;
static fixedpoint led_b = 0;

static fixedpoint delta_r;
static fixedpoint delta_g;
static fixedpoint delta_b;

static const struct remote_key *last_remote_command;

static unsigned long ramp_start_time;
static unsigned long ramp_last_step;

enum ramp_name {
	OFF,
	FADE7,
	FADE7_END = 7,
	ERROR = FADE7_END + 1,
	ERROR_END = ERROR + 3,
};

static uint8_t doing_ramp;
static uint8_t ramp_is_paused;

struct ramp {
	int time;
	float r;
	float g;
	float b;
	int next_ramp;
};

const int dur = 5000; 

const struct ramp LED_ramps[] = {
		{ 1, 	   0, 	   0, 	   0, 0 }, //OFF
		{ dur,	 1.0,	   0,	   0, 2 }, //FADE7
		{ dur/2,	 0.0,	1.0,	   0, 3 },
		{ dur,	 0.0,	0.0,	 1.0, 4 },
		{ dur,	 1.0,	1.0,	 0.0, 5 },
		{ dur,	 1.0,	0.0,	 1.0, 6 },
		{ dur,	 0.0,	1.0,	 1.0, 7 },
		{ dur,	 1.0,	1.0,	 1.0, 1 }, //FADE7_END
		{ 1,	   0,	  0,       0, ERROR+1 },     //ERROR
		{ 1000,	   0,	  0,       0, ERROR+2 },     //ERROR
		{ 1,	 1.0,	  0,       0, ERROR+3 },     //ERROR
		{ 1000,	 1.0,	  0,       0, ERROR },     //ERROR_END

};

/* Strobe */
static uint8_t strobe_mode;
static fixedpoint strobe_target_r = 0;
static fixedpoint strobe_target_g = 0;
static fixedpoint strobe_target_b = 0;
int strobe_period = 500;

/* --End of data definition-- */

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
	if (which == OFF) {
		// OFF is a special ramp that shuts everything right away.
		set_led(0, 0, 0);
	}
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

void strobe(void) 
{
	if ((millis() / strobe_period) & 1) {
		set_led(0, 0, 0);
	} else {
		set_led(strobe_target_r, strobe_target_g, strobe_target_b);
	}
}

const struct remote_key *lookup_code(uint32_t key) 
{
	int i, j;

	for (i = 0; i < NROWS; i++) {
		for (j = 0; j < NCOLS; j++) {
			if (remote_codes[i][j].code == key) {
				return &remote_codes[i][j];
			}
		}
	}
	return &unknown_key;
}


void setup(){
	printf_begin();
	Serial.begin(57600);
	
	irrecv.enableIRIn(); // Start the receiver

	pinMode(DIAG_LED_PIN, OUTPUT);
	digitalWrite(DIAG_LED_PIN, 0);

	// fix PWM (LED flicker)
	bitSet(TCCR1B, WGM12);

	printf("Ready!\n");
}

void loop(){
	decode_results results;
	
	uint32_t code;
	const char *name;
	if (irrecv.decode(&results)) {
		code = results.value;
		if (code == 0xFFFFFFFF) { 
			// "repeat" code
			Serial.println("XXX implement repeat");
		}

		last_remote_command = lookup_code(code);
		printf("Command %s code %x\n", last_remote_command->name, code);

		if (last_remote_command->cb) {
			last_remote_command->cb(last_remote_command->cbdata);
		}
		irrecv.resume(); // Receive the next value
	}

	if (doing_ramp && !ramp_is_paused) {
		led_ramp();
	} else if (strobe_mode) {
		strobe();
	}
}

void remote_cb_light(void *bool_increase)
{ printf("Implement me %s\n", __FUNCTION__); return; }
void remote_cb_play(void *bool_play)
{
	int start = (int)bool_play;
	if (!start) {
		// OFF button
		start_ramp(OFF);
		return;
	} 

	// play/pause button
	if (doing_ramp) {
		ramp_is_paused = !ramp_is_paused;
	} else {
		start_ramp(FADE7);
	}
}

/** Set a specific color **/
void remote_cb_color(void *struct_color)
{
	// Retrieve color, stop current ramp, set leds
	const struct color *c = (const struct color *)struct_color;

	start_ramp(OFF);
	printf("Got color %d %d %d\n", (int)c->r, (int)c->g, (int)c->b);
	set_led(I2F((int32_t)c->r), I2F((int32_t)c->g), I2F((int32_t)c->b));
	
}

void remote_cb_NULL(void *arg)
	{ printf("Implement me %s\n", __FUNCTION__); return; }
void remote_cb_diy(void *int_number)
	{ printf("Implement me %s\n", __FUNCTION__); return; }
void remote_cb_strobe(void *none)
{
	// Toggle strobe mode (flash between current color and black)
	strobe_mode = !strobe_mode;
	strobe_target_r = led_r;
	strobe_target_g = led_g;
	strobe_target_b = led_b;
	strobe_period = 500;
}

void remote_cb_strobeperiod(void *bool_decrease)
{
	// Change strobing period
	int decrease = (int)bool_decrease;
	if (decrease) {
		strobe_period -= 50;
	} else {
		strobe_period += 50;
	}
}

void remote_cb_jump(void *int_number)
	{ printf("Implement me %s\n", __FUNCTION__); return; }

void remote_cb_fade(void *int_number)
{
	int variant = (int)int_number;
	if (variant == 7) {
		start_ramp(FADE7);
	} else {
		start_ramp(ERROR);
	}

}

void remote_cb_colortweak(void *char_color)
{ printf("Implement me %s\n", __FUNCTION__); return; }

