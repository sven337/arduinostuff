/*
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>
#include "remote_control.h"

int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(57600);
  irrecv.enableIRIn(); // Start the receiver
}

const char *lookup_code(uint32_t key, int *x, int *y) 
{
	int i, j;

	for (i = 0; i < NROWS; i++) {
		for (j = 0; j < NCOLS; j++) {
			if (remote_codes[i][j].code == key) {
				*y = i;
				*x = j;
				return remote_codes[i][j].func;
			}
		}
	}

	*x = -1;
	*y = -1;
	return "UNKNOWN";
}


void loop() {
	uint32_t code;
	int x, y;
	const char *func;
	if (irrecv.decode(&results)) {
		code = results.value;
		func = lookup_code(code, &x, &y);
		if (code != 0xFFFFFFFF) {
			// ignore "repeat" code
			Serial.println(func);

			if (x == -1) {
				Serial.println(code, HEX);
			}
		}
		irrecv.resume(); // Receive the next value
	}
	delay(100);
}
