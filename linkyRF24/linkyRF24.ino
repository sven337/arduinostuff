#include <SPI.h>
#include <avr/sleep.h>
#include <JeeLib.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"

const int CE_PIN = 9;
const int CSN_PIN = 8;
const int BATTERY_PIN = A3;
const int SOLAR_PIN = A0;
const int SOLAR_RESISTOR_PIN = A1;
const int LED_YELLOW = 5;
const int LED_RED = 4;

const uint64_t pipe_linky  = 0xF0F0F0F0F3LL;

#define PIPE_GAZ_ID 1
#define PIPE_MAILBOX_ID 3
#define PIPE_THERMOMETER_ID 4

#define startFrame 0x02
#define endFrame 0x03

#define HAS_SOLAR_PANEL 0
const uint64_t pipe_address = 0xF0F0F0F0F3LL;
RF24 rf24(CE_PIN, CSN_PIN);

SoftwareSerial edfSerial(2, 6); //RX, TX (unused TX)
char teleinfo_buf[255];
int teleinfo_cur = 0;

int8_t frame_bigskip_counter; // how many more frames to skip (for unimportant fields)
int8_t frame_smallskip_counter; // how many more frames to skip (for important fields)

#define BIGSKIP_INTERVAL 30 // count 30 full frames (approx 1 min)
#define SMALLSKIP_INTERVAL 3 
/*
DEMAIN ---- "
ADCO 000000000000 J
OPTARIF BBR( S
ISOUSC 30 9
BBRHCJB 033487837 H
BBRHPJB 006345478 O
BBRHCJW 000000000 2
BBRHPJW 000000000 ?
BBRHCJR 000000000 -
BBRHPJR 000000000 :
PTEC HPJB P
DEMAIN ---- "
IINST 003 Z
IMAX 090 H
PAPP 00000 !
HHPHC A ,
MOTDETAT 000000 B

ADCO 000000000000 J
OPTARIF BBR( S
ISOUSC 30 9
BBRHCJB 033487837 H
BBRHPJB 006345478 O
BBRHCJW 000000000 2
BBRHPJW 000000000 ?
BBRHCJR 000000000 -
BBRHPJR 000000000 :
PTEC HPJB P
DEMAIN ---- "
IINST 003 Z
//ADPS
IMAX 090 H
PAPP 00000 !
HHPHC A ,
MOTDETAT 000000 B

*/
void send_line()
{
    // We have four bytes on the air to fit the data. Build packets intelligently.
    const struct {
        const char     *name; // name of frame
        uint8_t   frame_type; // one-character code for frame, 0 if ignored
        uint8_t   pos_offset; // offset to the data value to send over the air
        bool   use_smallskip; // use big or small skip counter?
    } mapping[] = {
/*          { "ADCO",     0 , 0, 0}, //000000000000 J
            { "OPTARIF",  0 , 0, 0}, //BBR( S
            { "ISOUSC",   0 , 0, 0}, //30 9 */
            { "BBRHCJB", 'b', 0, 0}, //033487837 H
            { "BBRHPJB", 'B', 0, 0}, //006345478 O
            { "BBRHCJW", 'w', 0, 0}, //000000000 2
            { "BBRHPJW", 'W', 0, 0}, //000000000 ?
            { "BBRHCJR", 'r', 0, 0},  //000000000 -
            { "BBRHPJR", 'R', 0, 0}, //000000000 :
            { "PTEC",    'J', 1, 0}, //HPJB P
            { "DEMAIN",  'D', 0, 0}, //---- "
            { "IINST",   'I', 0, 1}, //003 Z
            { "ADPS",    'A', 0, 1},
//          { "IMAX",     0 , 0, 0}, //090 H
            { "PAPP",    'P', 1, 1}, //00000 !
/*          { "HHPHC",    0 , 0, 0}, //A , */
//            { "MOTDETAT", 0 , 0, 0}, //000000 B // KEEP THIS LAST! marks the end of a frame
    };

#define MATCH(X) !memcmp(teleinfo_buf, X, strlen(X))

    char *p = &teleinfo_buf[0];


    unsigned int i = 0;
    for (i = 0; i < sizeof(mapping)/sizeof(mapping[0]); i++) {
        if (!MATCH(mapping[i].name)) {
            continue;
        }

        // Ignore field if in holdoff counters
        if (mapping[i].use_smallskip) {
            if (frame_smallskip_counter > 0) {
                return;
            }
        } else {
            if (frame_bigskip_counter > 0) {
                return;
            }
        }

        uint8_t data[4];
        data[0] = mapping[i].frame_type;
        int pos = strlen(mapping[i].name) + 1;
        pos += mapping[i].pos_offset; // ignore the first N characters of the frame (1)

        if (MATCH("BBRH")) {
            uint32_t val = strtol(teleinfo_buf + pos, NULL, 10);
            // Cannot fit the whole index in 24 bits, so trim the 6 LSBs
            val = val >> 6;
            data[1] = (val >> 16) & 0xFF;
            data[2] = (val >>  8) & 0xFF;
            data[3] = val & 0xFF;
        }  else {
            memcpy(&data[1], teleinfo_buf + pos, 3);
        }

        // Data is ready to send, should it be sent?
        radio_send(data[0], data[1], data[2], data[3]);
    }
}

void consume_teleinfo()
{
    static bool inFrame = false;

    while (edfSerial.available()) {
        char c = edfSerial.read() & 0x7F;
        if (c == startFrame) {
            inFrame = true;
            teleinfo_cur = 0;
            teleinfo_buf[teleinfo_cur] = 0;
            continue;
        } else if (c == endFrame) {
            inFrame = false;
            teleinfo_buf[teleinfo_cur] = 0;
            teleinfo_cur = 0;
            // End of frame, adjust holdoff counters
            frame_bigskip_counter--;
            frame_smallskip_counter--;
            if (frame_bigskip_counter < 0) {
                frame_bigskip_counter = BIGSKIP_INTERVAL;
            } 
            if (frame_smallskip_counter < 0) {
                frame_smallskip_counter = SMALLSKIP_INTERVAL;
            }
            continue;
        }
        
        if (!inFrame) {
            continue;
        }

        if (c == '\n' || c == '\r') {
            teleinfo_buf[teleinfo_cur++] = '\n';
            teleinfo_buf[teleinfo_cur] = 0;
            if (teleinfo_cur > 1) {
                send_line();
            }
            teleinfo_cur = 0;
            continue;
        }
        teleinfo_buf[teleinfo_cur++] = c;
/*
        // Ignore fields
             "IMAX", 
             "ADCO", 
             "HHPHC",
             "ISOUSC", 
             "OPTARIF", 
             "MOTDETAT", 
        if (teleinfo_cur == 4) {
            if (!memcmp(teleinfo_buf, "IMAX", 4) ||
                !memcmp(teleinfo_buf, "ADCO", 4)) {
                // 1200baud = 6.66ms per character, sleep for a bit
                Sleepy::loseSomeTime(16);
                ignore_frame = 1;
            }
        }*/
    }
}

static int init_failed = 0;

ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

int radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
	uint8_t payload[4] = { p0, p1, p2, p3 };
	digitalWrite(LED_YELLOW, 1);
	delayMicroseconds(5000);
	bool ok = rf24.write(payload, 4);
	if (ok) 
		Serial.println("send ok");
	else
		Serial.println("send KO");
	
    printf("sending %c %c %c %c\n", p0, p1, p2, p3);
	digitalWrite(LED_YELLOW, 0);

	if (!ok) {
		digitalWrite(LED_RED, 1);
		return -1;
	}
	digitalWrite(LED_RED, 0);
	return 0;
}


int radio_send_bat(uint16_t bat)
{
//	return radio_send('B', thermometer_identification_letter, (bat >> 8) & 0xFF, bat & 0xFF);
    return 0;
}

int radio_send_panel(uint16_t panel_voltage)
{
//	return radio_send('S', thermometer_identification_letter, (panel_voltage >> 8) & 0xFF, panel_voltage & 0xFF);
    return 0;
}

int radio_send_current(uint16_t panel_voltage, uint16_t panel_resistor_voltage)
{
	// Panel -(-> SOLAR_PIN) - resistor (-> SOLAR_RESISTOR_PIN) - circuit
	// So the current is the voltage across the resistor, which is panel_voltage - panel_resistor_voltage
	// It's not supposed to be negative but I've observed -1 in some cases, so make the difference signed.
	int16_t value = panel_voltage - panel_resistor_voltage;

	// Resistor is set to 22 ohm
	// U = R.I so I = U / 22, send it *1000 to see something
	value = 1000 * (value  * 3.3f / 1024) / 22;
//	return radio_send('C', thermometer_identification_letter, (value >> 8) & 0xFF, value & 0xFF);
    return 0;
}

void setup(){
	printf_begin();
	Serial.begin(57600);
    edfSerial.begin(1200); 

	// Radio init
	rf24.begin();
	rf24.powerDown();
	rf24.setRetries(15, 15);
	rf24.setChannel(80);
	rf24.setCRCLength(RF24_CRC_16);
	rf24.setPayloadSize(sizeof(unsigned long));
	rf24.setPALevel(RF24_PA_MAX);
	rf24.setDataRate(RF24_250KBPS);
 	rf24.setAutoAck(true);
	rf24.openWritingPipe(pipe_address);
	rf24.openReadingPipe(1, pipe_address);

	rf24.printDetails();

	if ((rf24.getDataRate() != RF24_250KBPS) ||
		(rf24.getCRCLength() != RF24_CRC_16)) {
		// failed to initialize radio
		init_failed = 1;
	}


	pinMode(LED_YELLOW, OUTPUT);
	pinMode(LED_RED, OUTPUT);
	digitalWrite(LED_YELLOW, 1);
	digitalWrite(LED_RED, 1);
	delay(100);
	digitalWrite(LED_RED, 0);
	delay(100);
	digitalWrite(LED_RED, 1);
	delay(100);
	digitalWrite(LED_RED, 0);
	delay(100);
	digitalWrite(LED_RED, 1);
	delay(100);
	digitalWrite(LED_RED, 0);
	digitalWrite(LED_YELLOW, 0);

	rf24.powerDown();
}

float battery_voltage(uint16_t battery_level)
{
	float lvl = (float)battery_level * 3.3 / 65536.0;
	return lvl;
}

void loop() 
{
	int red = 0;
	while (init_failed) {
			digitalWrite(LED_RED, red);
			red = !red;
			// Christmas tree if init failed
			digitalWrite(LED_YELLOW, 1);
			delay(50);
			digitalWrite(LED_YELLOW, 0);
			delay(50);
			digitalWrite(LED_YELLOW, 1);
			delay(50);
			digitalWrite(LED_YELLOW, 0);
			delay(50);
	}
    
    if (edfSerial.available()) {
        consume_teleinfo();
    }

	uint16_t battery_level = analogRead(BATTERY_PIN);
#if HAS_SOLAR_PANEL
	uint16_t panel_voltage = analogRead(SOLAR_PIN);
	uint16_t resistor_voltage = analogRead(SOLAR_RESISTOR_PIN);
#endif
	rf24.powerUp();

	bool fail = radio_send_bat(battery_level);
#if HAS_SOLAR_PANEL
	fail &= radio_send_panel(panel_voltage);
	fail &= radio_send_current(panel_voltage, resistor_voltage);
#endif
/*
	i = 3;

	while (i--) {
	    fail = radio_send('T', thermometer_identification_letter, (raw >> 8) & 0xFF, raw & 0xFF);
		if (!fail) {
			break;
		}
        // Indicate failure to receive ACK
        fail = radio_send('F', thermometer_identification_letter, i, 0);
		Sleepy::loseSomeTime(2048L);
	}
    */
    /*
sleep:	
#if HAS_RF24
	rf24.powerDown();
#endif
	Serial.println("going to sleep now");
	Serial.flush();
	Sleepy::loseSomeTime(32768L);
	Serial.println("waking up");

	for (int i = 1; i < 15L*60L*1000L / 32768L; i++) {
#if HAS_SOLAR_PANEL
		if (battery_voltage(battery_level) >= 1.4f) {
			// Battery is overcharged, try to waste energy!
			delay(10000);
		}
#endif
		if (!Sleepy::loseSomeTime(32768L)) {
			Serial.println("woken up by intr");
		}
	}*/
}



