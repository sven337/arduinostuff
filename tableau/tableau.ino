#include <Arduino.h>
#include <EEPROM.h>
#include <IRremote.h>
#include "printf.h" 
#include "remote_control.h"

/* Pins */
const int DIAG_LED_PIN = 7; // P3 D
const int R = 9;
const int G = 5;
const int B = 6;
const int RECV_PIN = 11;

/* Working mode */
enum mode {
	OFF = 0,
	RAMP,
	PAUSE,
	FIXED,
	STROBE,
	SAVE_CONFIRM,
} current_mode, previous_mode;

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

enum ramp_name {
	FADE7 = 1,
	FADE7_END = 7,
	ERROR = FADE7_END + 1,
	ERROR_END = ERROR + 3,
	SUNRISE = ERROR_END + 1,
	SUNSET = SUNRISE + 8,
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
				  { 1, 	   0, 	   0, 	   0, 0 }, //OFF
		[FADE7] = { dur,	 0.0,	0.0,	 1.0, FADE7+1 }, //FADE7
				{ dur,	 0.0,	1.0,	 1.0, FADE7+2 },
				{ dur/2,	 0.0,	1.0,	   0, FADE7+3 },
				{ dur,	 1.0,	1.0,	 0.0, FADE7+4 },
				{ dur,	 1.0,	   0,	   0, FADE7+5 },
				{ dur,	 1.0,	0.0,	 1.0, FADE7+6 },
		[FADE7_END] = { dur/2,	 0.7,	0.7,	 0.7, FADE7 }, //FADE7_END
		[ERROR] ={ 0,	   0,	  0,       0, ERROR+1 },     //ERROR
				{ 1000,	   0,	  0,       0, ERROR+2 },     //ERROR
				{ 0,	 1.0,	  0,       0, ERROR+3 },     //ERROR
		[ERROR_END] = { 1000,	 1.0,	  0,       0, ERROR },     //ERROR_END
		[SUNRISE] = { 0,         0,    0,    0.01, SUNRISE+1},    // SUNRISE
					{ 300000,       0,    0,    0.02, SUNRISE+2}, // Dark blue
					{ 900000,  0.05,    0.02,    0.05, SUNRISE+3},
					{ 1500000,  0.10,    0.02,    0.05, SUNRISE+4}, // Sun begins to rise
					{ 1800000,  0.20,   0.10,    0.05, SUNRISE+5},
					{ 2100000,   0.50,   0.12,    0.05, SUNRISE+6}, // Yellowish
					{ 2400000,   0.60,   0.4,   0.2, SUNRISE+7}, // White
					{ 3600000,   1.00,   0.65,   0.45, OFF}, // Bright white
		[SUNSET] =  { 0,   1.00,   0.70,   0.0, SUNSET+1}, // SUNSET
					{ 600000L,   0.8,   0.04,   0.0, SUNSET+2},
					{ 600000L,   0.0,   0.00,   0.0, OFF},
};


/* Strobe */
static uint8_t strobe_target_r = 0;
static uint8_t strobe_target_g = 0;
static uint8_t strobe_target_b = 0;
int strobe_period = 500;

/* Remote control */

static const struct remote_key *last_remote_command;
IRrecv irrecv(RECV_PIN);

/* --End of data definition-- */

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

void strobe(void) 
{
	if ((millis() / strobe_period) & 1) {
		set_led(0, 0, 0);
	} else {
		set_led(I2F((uint32_t)strobe_target_r), I2F((uint32_t)strobe_target_g), I2F((uint32_t)strobe_target_b));
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
	return NULL;
}


void setup(){
	printf_begin();
	Serial.begin(57600);
	
	irrecv.enableIRIn(); // Start the receiver

	pinMode(DIAG_LED_PIN, OUTPUT);
	digitalWrite(DIAG_LED_PIN, 0);

	// fix PWM (LED flicker)
	bitSet(TCCR1B, WGM12);

	current_mode = OFF;
	previous_mode = OFF;
}

void loop(){
	decode_results results;
	
	uint32_t code;
	if (irrecv.decode(&results)) {
		code = results.value;

		if (code != 0xFFFFFFFF) {
			// Valid code, 0xFFFFFFFF is repeat = reuse previous command
			last_remote_command = lookup_code(code);
		} 
		if (last_remote_command && last_remote_command->cb) {
			last_remote_command->cb(last_remote_command->cbdata);

			// Blink to acknowledge command
			set_led(~led_r, ~led_g, ~led_b);
			delay(200);
			set_led(~led_r, ~led_g, ~led_b);
		}
		irrecv.resume(); // Receive the next value
	}

	switch (current_mode) {
		case OFF:
			if (led_r != 0 && led_g != 0 || led_b != 0) {
				set_led(0, 0, 0);
			}
			break;
		case RAMP:
			led_ramp();
			break;
		case PAUSE:
		case FIXED:
			break;
		case SAVE_CONFIRM:
			if (!((millis() >> 10) & 1)) {
				set_led(0, I2F((uint32_t)255), I2F((uint32_t)255));
			} else {
				set_led(0, 0, I2F((uint32_t)255));
			}
			break;
		case STROBE:
			strobe();
			break;
		default:
			start_ramp(ERROR);
	}
}

void change_mode(int newmode)
{
	previous_mode = current_mode;
	current_mode = (enum mode)newmode;
}

void remote_cb_light(void *bool_increase)
{ printf("Implement me %s\n", __FUNCTION__); return; }

void remote_cb_play(void *bool_play)
{
	int start = (int)bool_play;
	if (!start) {
		// OFF button
		change_mode(OFF);
		set_led(0, 0, 0);
		return;
	} 

	// play/pause button
	if (current_mode == OFF || current_mode == PAUSE) {
		change_mode(previous_mode);
	} else {
		change_mode(PAUSE);
	}
}

/** Set a specific color **/
void remote_cb_color(void *struct_color)
{
	// Retrieve color, stop current ramp, set leds
	const struct color *c = (const struct color *)struct_color;

	change_mode(FIXED);
	set_led(I2F((int32_t)c->r), I2F((int32_t)c->g), I2F((int32_t)c->b));
	
}

void remote_cb_NULL(void *arg)
	{ printf("Implement me %s\n", __FUNCTION__); return; }

void remote_cb_diy(void *int_number)
{
	int slot = (int)int_number;
	struct color col;

	if (current_mode == SAVE_CONFIRM) {
		// Confirming save: write current color to slot
		col.r = F2I(led_r);
		col.g = F2I(led_g);
		col.b = F2I(led_b);
		EEPROM.put(slot * sizeof(col), col);
		set_led(led_r, led_g, led_b);
	} else {
		if (current_mode == FIXED) {
			// Ask for confirmation to save the color?
			change_mode(SAVE_CONFIRM);
		} else {
			// Recall fixed color from slot
			change_mode(FIXED);
			EEPROM.get(slot * sizeof(col), col);
			led_r = I2F((uint32_t)col.r);
			led_g = I2F((uint32_t)col.g);
			led_b = I2F((uint32_t)col.b);
			set_led(led_r, led_g, led_b);
		}
	}
}

void remote_cb_strobe(void *none)
{
	// Toggle strobe mode (flash between current color and black)
	if (current_mode == STROBE) {
		change_mode(OFF);
	} else {
		change_mode(STROBE);
	}
	strobe_target_r = F2I(led_r);
	strobe_target_g = F2I(led_g);
	strobe_target_b = F2I(led_b);
	printf("strobe target r %d g %d b %d\n", strobe_target_r, strobe_target_g, strobe_target_b);
	strobe_period = 500;
}

void remote_cb_strobeperiod(void *bool_decrease)
{
	// Change strobing period
	uint8_t decrease = *(uint8_t *)(&bool_decrease);
	if (decrease) {
		if (strobe_period >= 100) {
			strobe_period -= 50;
		} else {
			strobe_period -= 5;
		}
	} else {
		strobe_period += 50;
	}
}

void remote_cb_jump(void *int_number)
	{ printf("Implement me %s\n", __FUNCTION__); return; }

void remote_cb_fade(void *int_number)
{
	uint8_t variant = *(uint8_t *)(&int_number);
	if (variant == 7) {
		start_ramp(FADE7);
	} else {
		start_ramp(ERROR);
	}

}

void remote_cb_colortweak(void *char_color)
{ 
	char cmd = *(char *)(&char_color);
	fixedpoint add = I2F((uint32_t)10);
	current_mode = FIXED;

	switch (cmd) {
		case 'R':
			led_r += add;
			break;
		case 'r':
			led_r -= add;
			break;
		case 'G':
			led_g += add;
			break;
		case 'g':
			led_g -= add;
			break;
		case 'B':
			led_b += add;
			break;
		case 'b':
			led_b -= add;
			break;
		default:
			start_ramp(ERROR);
			return;
	}

	set_led(led_r, led_g, led_b);
}

void remote_cb_auto(void *dummy)
{
	// Use "auto" button to start sunset sequence
	start_ramp(SUNSET);
}

