#include <PinChangeInt.h>

const int BUZZER_PIN = 4; //P1
const int BUZZER_LED_PIN = 6; //P3

static int got_buzzer_intr;
static long buzzer_tstamps[2];

static int alarm_now = 0;

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
	if (alarm_now) {
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

/*	Serial.print("Waiting for ");
	Serial.print(buzzer_wait_for);
	Serial.print(" got ");
	Serial.println(time);*/

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

void alarm_now(void)
{
	alarm_now = 1;
	digitalWrite(BUZZER_LED_PIN, HIGH);
}

void setup(){
	//start serial connection
	Serial.begin(9600);
	//configure pin2 as an input and enable the internal pull-up resistor
	pinMode(BUZZER_PIN, INPUT);
	pinMode(BUZZER_LED_PIN, OUTPUT); 
	PCintPort::attachInterrupt(BUZZER_PIN, &buzzer_interrupt, CHANGE);
	digitalWrite(BUZZER_LED_PIN, LOW);
}

void loop(){
	if (got_buzzer_intr) {
		long time = buzzer_tstamps[1] - buzzer_tstamps[0];

		if (buzzer_handle_silence_time(time)) {
			alarm_now();
		}

		got_buzzer_intr = 0;
	}
}



