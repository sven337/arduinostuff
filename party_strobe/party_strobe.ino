#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

#define LOG_OUT 0
#define OCTAVE 1
#define FHT_N 256
#define OCT_NORM 0
#include <FHT.h>

const int ON_OFF_PIN = 7;
const int BUTTON_PIN = 8;
const int SPEED_PIN = A3;
const int SOUND_PIN = A0;

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

	enum operation { ERROR, RISE, SLOW_DECAY, FAST_DECAY };

	struct {
		int magidx;
		int bias;
		// int avgidx is equal to i;
		enum operation op;
		float factor;
	} channels[] = {
			{ oct_low , 0, ERROR },
			{ oct_mid , 0, ERROR },
			{ oct_high, 0, ERROR },
	};

	for (int i = 0; i < 3; i++) {
		int bias = channels[i].bias;
		int magidx = channels[i].magidx;
		int diff;

		// Remove constant component and remap to 0..255	

		fht_oct_out[magidx] = map(fht_oct_out[magidx] - bias, 0, 255-bias, 0, 255);
	
		// Compare to moving average and make a decision based on that.
		// 		value < avg: decay (faster as value is lower)
		// 		value = avg: slow decay
		// 		value > avg: rise exponentially
		
		diff = fht_oct_out[magidx] - freq_avg_mag[i];
		if (abs(diff) < 5) {
			// Roughly equal, so slow decay
			channels[i].op = SLOW_DECAY;
		} else if (diff < 0) {
			channels[i].op = FAST_DECAY;
		} else {
			channels[i].op = RISE;
		}

		channels[i].factor = constrain(abs(diff)/40.0, 0.1, 1.0);

		// Update moving average
		freq_avg_mag[i] = (3 * freq_avg_mag[i] + fht_oct_out[magidx]) / 4;
	}

	int new_led[3];
	for (int i = 0; i < 3; i++) {
		new_led[i] = (i == 0) ? led_b : ((i == 1) ? led_g : led_r);
		switch (channels[i].op) {
			case SLOW_DECAY:
				if (new_led[i] < 5 * channels[i].factor) {
					new_led[i] = 0;
				} else {
					new_led[i] -= 5 * channels[i].factor;
				}
				break;
			case FAST_DECAY:
				if (new_led[i] < 10 * channels[i].factor) {
					new_led[i] = 0;
				} else {
					new_led[i] -= 10 * channels[i].factor;
				}
				break;
			case RISE:
				if (new_led[i] > 255 - 10 * channels[i].factor) {
					new_led[i] = 255;
				} else {
					new_led[i] += 10 * channels[i].factor;
				}
				break;
		}
	}	

   printf("%d %d %d (avg %d %d %d) op=(%d %d %d) ->r %d g %d b %d\n", fht_oct_out[oct_low], fht_oct_out[oct_mid], fht_oct_out[oct_high], freq_avg_mag[0], freq_avg_mag[1], freq_avg_mag[2], channels[0].op, channels[1].op, channels[2].op, new_led[2], new_led[1], new_led[0]);

   set_led(new_led[2], new_led[1], new_led[0]);


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



