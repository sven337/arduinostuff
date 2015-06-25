#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

#define LOG_OUT 0
#define OCTAVE 1
#define FHT_N 256
#define OCT_NORM 1
#include <FHT.h>

const int ON_OFF_PIN = 7;
const int BUTTON_PIN = 8;
const int SPEED_PIN = A3;
const int SOUND_PIN = A0;

const int R = 5;
const int G = 9;
const int B = 6;

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

static uint8_t led_r = 0;
static uint8_t led_g = 0;
static uint8_t led_b = 0;

static fixedpoint led_r_fp;
static fixedpoint led_g_fp;
static fixedpoint led_b_fp;

static fixedpoint delta_r;
static fixedpoint delta_g;
static fixedpoint delta_b;

static unsigned long ramp_start_time;
static unsigned long ramp_last_step;

bool change_mode_button_pressed;

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

// Average magnitudes for frequency bins
uint8_t freq_avg_mag[3];

unsigned int hue = 0;
unsigned int value = 0;

void on_off_interrupt(void)
{
	// Potentiometer on/off enables/disables strobe from music mode
	strobe_off = PCintPort::pinState;
}

void button_interrupt(void)
{
	change_mode_button_pressed = true;
}

uint8_t bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (uint8_t)(255.0 * (exp(0.6931471805599453 / 255.0 * in) - 1.0));
//	return (uint8_t)(255.0 * in);
}

void set_led(uint8_t r, uint8_t g, uint8_t b)
{
	led_r = r;
	led_g = g;
	led_b = b;

	uint8_t my_r, my_g, my_b;
#define BRI_CORRECT(X) (bri(X) & 0xFF)
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

	led_r_fp = I2F((int32_t)led_r);
	led_g_fp = I2F((int32_t)led_g);
	led_b_fp = I2F((int32_t)led_b);
	delta_r = (I2F((int32_t)(255.0 * r->r)) - led_r_fp) / r->duration;
	delta_g = (I2F((int32_t)(255.0 * r->g)) - led_g_fp) / r->duration;
	delta_b = (I2F((int32_t)(255.0 * r->b)) - led_b_fp) / r->duration;

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
			set_led((uint8_t)(255.0 * r->r), (uint8_t)(255.0 * r->g), (uint8_t)(255.0 * r->b));
			start_ramp(r->next_ramp);
		}
		return;
	}

	unsigned long this_step_duration = now - ramp_last_step;
	ramp_last_step = now;

	led_r_fp = led_r_fp + this_step_duration * delta_r;
	led_g_fp = led_g_fp + this_step_duration * delta_g;
	led_b_fp = led_b_fp + this_step_duration * delta_b;

	printf("ramp r %ld g %ld b %ld\n", led_r_fp, led_g_fp, led_b_fp);
	set_led(F2I(led_r_fp), F2I(led_g_fp), F2I(led_b_fp));
}

void strobe()
{
	static int last_wait = 0;
	static uint8_t next = 0;
	static long next_at = 0;

	long now = millis();
	int wait = constrain(13 + 15 / sequence_speed_factor, 1, 500);

	float freq = 1 + sequence_speed_factor * 30;
	wait = 1000.0 / freq;
	if (wait != last_wait) {
		last_wait = wait;
	}

	if (now > next_at) {
		set_led(next, next, next);
		next_at = now + wait;
		next = 255 - next;
	} else {
		if (next_at - now > wait)
			next_at = now + wait;
	}
}

/******************************************************************************
  This function converts HSV values to RGB values, scaled from 0 to maxBrightness

  The ranges for the input variables are:
  hue: 0-360
  sat: 0-255
  lig: 0-255

  The ranges for the output variables are:
  r: 0-maxBrightness
  g: 0-maxBrightness
  b: 0-maxBrightness
  
  r,g, and b are passed as pointers, because a function cannot have 3 return variables
  Use it like this:
  int hue, sat, val; 
  unsigned char red, green, blue;
  // set hue, sat and val
  hsv2rgb(hue, sat, val, &red, &green, &blue, maxBrightness); //pass r, g, and b as the location where the result should be stored
  // use r, b and g.
 
  (c) Elco Jacobs, E-atelier Industrial Design TU/e, July 2011.
  https://code.google.com/p/shiftpwm/source/browse/trunk/examples/ShiftPWM_Example1/hsv2rgb.cpp?r=3
  
 *****************************************************************************/


void hsv2rgb(unsigned int hue, unsigned int sat, unsigned int val, 
              unsigned char * r, unsigned char * g, unsigned char * b) { 
    unsigned int H_accent = hue/60;
    unsigned int bottom = ((255 - sat) * val)>>8;
    unsigned int top = val;
    unsigned char rising  = ((top-bottom)  *(hue%60   )  )  /  60  +  bottom;
    unsigned char falling = ((top-bottom)  *(60-hue%60)  )  /  60  +  bottom;
    
    switch(H_accent) {
        case 0:
                *r = top;
                *g = rising;
                *b = bottom;
        break;
        
        case 1:
                *r = falling;
                *g = top;
                *b = bottom;
        break;
        
        case 2:
                *r = bottom;
                *g = top;
                *b = rising;
        break;
        
        case 3:
                *r = bottom;
                *g = falling;
                *b = top;
        break;
        
        case 4:
                *r = rising;
                *g = bottom;
                *b = top;
        break;
        
        case 5:
                *r = top;
                *g = bottom;
                *b = falling;
        break;
    }
}


void music()
{
	const int oct_low = 2;
	const int oct_mid = 4;
	const int oct_high = 6;

	int timer = TIMSK0;
	int adcsra = ADCSRA;
	ADMUX = 0x40; // use adc0
	DIDR0 = 0x01; // turn off the digital input for adc0
	ADCSRA = 0xe5; // set the adc to free running mode
	TIMSK0 = 0;
	// FFT and set leds accordingly
	cli();  // UDRE interrupt slows this way down on arduino1.0
	for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
		while(!(ADCSRA & 0x10)); // wait for adc to be ready
		ADCSRA = 0xf5; // restart adc
		byte m = ADCL; // fetch adc data
		byte j = ADCH;
		int k = (j << 8) | m; // form into an int
		k -= 0x0200; // form into a signed int
		k <<= 6; // form into a 16b signed int
		fht_input[i] = k; // put real data into bins
	}
	fht_window(); // window the data for better frequency response
	fht_reorder(); // reorder the data before doing the fht
	fht_run(); // process the data in the fht
	fht_mag_octave(); // take the output of the fht
	sei();
	TIMSK0 = timer;
	ADCSRA = adcsra;

	Serial.write(255);
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 16; j++) {
			Serial.write(fht_oct_out[i]);
		}
	}	

	// Bass (oct_low) controls value with a fade
	float diff = fht_oct_out[oct_low] - freq_avg_mag[0];
	float factor = constrain(abs(diff) / 35.0, 0.01, 3.0);
	if (abs(diff) < 5) {
		// Roughly equal, so slow decay
		if (value > 15) {
			value -= 15;
		} else {
			value = 0;
		}
	} else if (diff < 0) {
		if (value < 75.0 * factor) {
			value = 0;
		} else {
			value -= 75.0 * factor;
		}
	} else {
		if (value > 255.0 - 100.0 * factor) {
			value = 255;
		} else {
			value += 100.0 * factor;
		}
	}
	// Update moving average
	freq_avg_mag[0] = (2 * freq_avg_mag[0] + fht_oct_out[oct_low]) / 3;

	// High freqs control hue
	hue += abs(fht_oct_out[oct_high] - freq_avg_mag[2]);
	hue %= 360;
	freq_avg_mag[2] = (2 * freq_avg_mag[2] + fht_oct_out[oct_high]) / 3;

	uint8_t r, g, b;
	hsv2rgb(hue, 255, value, &r, &g, &b);
	set_led(r, g, b);


}

void setup()
{
	//start serial connection
	printf_begin();
	Serial.begin(115200);
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

	current_mode = MUSIC;
//	start_ramp(FADE7);
	ADMUX = 0x40; // use adc0
	DIDR0 = 0x01; // turn off the digital input for adc0
//	TIMSK0 = 0; // turn off timer0 for lower jitter

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

	if (change_mode_button_pressed) {
		if (current_mode == STROBE) {
			current_mode = OFF;
		} else {
			current_mode = (enum mode)(current_mode + 1);
			if (current_mode == RAMP) {
				start_ramp(FADE7);
			}
		}
		change_mode_button_pressed = false;
		printf("current mode is %d\n", current_mode);
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



