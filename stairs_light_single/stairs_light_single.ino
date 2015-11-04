#include <SPI.h>
#include <PinChangeInt.h>
#include "printf.h" 

const int PIR_PIN1 = 7;
const int PIR_PIN2 = 8;

const int LED = 5;
const int DIAG_LED = 13;

static int got_pir_intr = 0;
static unsigned long presence_detected_at;

struct ramp {
	unsigned long time;
	signed int l;
	int next_ramp;
};

typedef signed long fixedpoint;

#define I2F(I) (I << 20)
#define F2I(F) (F >> 20)

static fixedpoint led_l = 0;

static fixedpoint delta_l;

static unsigned long ramp_start_time;
static unsigned long ramp_last_step;

enum ramp_name {
	OFF,
	RISE_START,
	RISE_END = 3,
	FADEOUT_START,
	FADEOUT_END = 5,
};

static uint8_t doing_ramp;

const struct ramp LED_ramps[] = {
		{ 1, 	   0, 0 }, //OFF
		{ 1,	   4, 2 }, //RISE_START
		{ 200,    25, RISE_END },
		{ 1000,	 255, RISE_END }, // RISE_END	
		{ 1000,   25, FADEOUT_END }, // FADEOUT_START
		{ 4000,    0, FADEOUT_END }, // FADEOUT_END
};

uint8_t bri(float in)
{
	// Maps linear 0 -> 1.0 to logarithmic 0.0 -> 1.0
	// Magic value is ln(2) 
	return (uint8_t)(255.0 * (exp(0.6931471805599453 / 255.0 * in) - 1.0));
}

void set_led(long l) //fixedpoint!
{
	led_l = l;

	uint8_t my_l;
#define BRI_CORRECT(X) (bri(F2I(X)) & 0xFF)
	my_l = BRI_CORRECT(l);
	printf("set led to %d (%ld)\n", my_l, l);
	analogWrite(LED, my_l);
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
	delta_l = (I2F((int32_t)(r->l)) - led_l) / (signed)r->time;

	printf("Starting ramp %d (r->l - led_l) = %ld, delta is l %ld\n", doing_ramp, I2F((int32_t)(r->l) - led_l), delta_l);
}

void led_ramp(void)
{
	const struct ramp *r = &LED_ramps[doing_ramp];
	const unsigned long now = millis();
	unsigned long delay_in_ramp = now - ramp_start_time;
	if (delay_in_ramp > r->time) {
		// Ramp is done, move on
		if (r->next_ramp != doing_ramp) {
			set_led(I2F((int32_t)(r->l)));
			start_ramp(r->next_ramp);
		} 
		return;
	}

	unsigned long this_step_duration = now - ramp_last_step;
	ramp_last_step = now;

	printf("new value  %ld\n", led_l + this_step_duration + delta_l);
	set_led(led_l + this_step_duration * delta_l);
}

void setup(){
	//start serial connection
	printf_begin();
	Serial.begin(57600);
	pinMode(PIR_PIN1, INPUT);
	pinMode(PIR_PIN2, INPUT);
	PCintPort::attachInterrupt(PIR_PIN1, &pir_interrupt, CHANGE);
	PCintPort::attachInterrupt(PIR_PIN2, &pir_interrupt, CHANGE);

	// Trigger false interrupt reading on PIR pin to not miss its
	// current state
	got_pir_intr = 1;

	set_led(0);

	start_ramp(OFF);
	printf("Ready!\n");
}

void loop(){
	if (got_pir_intr) {
		if ((digitalRead(PIR_PIN1) || digitalRead(PIR_PIN2)) && !presence_detected_at) {
			// Detected someone
			presence_detected_at = millis();
			printf("Human here\n");
			start_ramp(RISE_START);
		} else {
			printf("PIR doesn't see us any longer.\n");
		}
		got_pir_intr = 0;
	}
	
	if (presence_detected_at && (millis() - presence_detected_at > 15000)) {
		// Timeout on presence
		presence_detected_at = 0;
		start_ramp(FADEOUT_START);
	}

	led_ramp();
}



