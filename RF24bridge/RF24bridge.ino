#include <SPI.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"

const int CE_PIN = 9;
const int CSN_PIN = 8;
const int LED_YELLOW = 5;
const int LED_RED = 4;
const int DS18B20_PIN = 7;

const uint64_t pipe_gaz 	 = 0xF0F0F0F0F0LL;
const uint64_t pipe_mailbox  = 0xF0F0F0F0F3LL;
const uint64_t pipe_thermometer  = 0xF0F0F0F0F4LL;

#define PIPE_GAZ_ID 1
#define PIPE_MAILBOX_ID 3
#define PIPE_THERMOMETER_ID 4

#define startFrame 0x02
#define endFrame 0x03

#define HAS_RF24 1
RF24 rf24(CE_PIN, CSN_PIN, 4000000);

OneWire ds(DS18B20_PIN);
uint32_t send_next_temperature_at = 1000;
const uint8_t thermometer_identification_letter = 'R'; // "rochelle"

struct {
    uint8_t addr[8];
    uint8_t letter;
} thermometer_letter_from_addr[] = {
        {{ 0x28, 0xff, 0xbd, 0xd3, 0x90, 0x15, 0x03, 0xbb }, 'R'}, //rochelle (self)
        {{ 0x28, 0x21, 0x5e, 0x79, 0xa2, 0x00, 0x03, 0xc8 }, 'B'}, //bedroom
        {{ 0x28, 0x05, 0x46, 0x79, 0xa2, 0x00, 0x03, 0xe0 }, 'L'}, //living
        {{ 0x28, 0xd0, 0x41, 0x79, 0xa2, 0x00, 0x03, 0x9e }, 'K'}, //kid
};

char serial_cmd[255];
int serial_cmd_cur = 0;

int send_rf24_cmd(uint64_t addr, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3)
{
	uint8_t payload[4];
	int ret = -1;
	payload[0] = param0;
	payload[1] = param1;
	payload[2] = param2;
	payload[3] = param3;

#if HAS_RF24
	printf("send rf24...");
	rf24.stopListening();
	digitalWrite(LED_YELLOW, 1);
	delayMicroseconds(10000);
	rf24.openWritingPipe(addr);
	rf24.powerUp();
	delayMicroseconds(10000);
	bool ok = rf24.write(&payload[0], 4);

	digitalWrite(LED_YELLOW, 0);
	if (ok) {
		printf("... successful\n");
		ret = 0;
	} else {
		digitalWrite(LED_RED, 1);
		printf("... could not send RF24 cmd\n");
	}
	rf24.startListening();
	return ret;
#endif
}

void setup()
{
    printf_begin();
    Serial.begin(115200);
#if HAS_RF24
	rf24.begin();
	rf24.powerDown();
	rf24.setRetries(15, 15);
	rf24.setChannel(80);
	rf24.setCRCLength(RF24_CRC_16);
	rf24.setPayloadSize(sizeof(unsigned long));
	rf24.setPALevel(RF24_PA_MAX);
	rf24.setDataRate(RF24_250KBPS);
 	rf24.setAutoAck(true);

    rf24.openReadingPipe(PIPE_GAZ_ID, pipe_gaz);
    rf24.openReadingPipe(PIPE_MAILBOX_ID, pipe_mailbox);
    rf24.openReadingPipe(PIPE_THERMOMETER_ID, pipe_thermometer);
	rf24.startListening();

	rf24.printDetails();

	if ((rf24.getDataRate() != RF24_250KBPS) ||
		(rf24.getCRCLength() != RF24_CRC_16) /*|| 
		(rf24.getChannel() != 95)*/) {
        printf("Failed to initialize radio\n");
	}
#endif	
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
}

void consume_rf24()
{
    uint8_t p[4];
	uint8_t pipe = 1;

	 while (rf24.available(&pipe)) {
		rf24.read(p, 4);
        bool decoded = true;
        /*
        if (p[0] == 'B') {
            uint16_t value = p[2] << 8 | p[3];
            float volt = value*3.3f/1024; //no voltage divider so no 2*
            float level = (volt-0.8)/(1.5-0.8); //boost cutoff at 0.8
            printf("Thermometer %c battery level: %ddV = %d%%\r\n", p[1], (int)(volt * 10.0), (int)(100.0*level));
        } else if (p[0] == 'T') {
            int16_t temperature = p[2] << 8 | p[3];
            printf("Thermometer %c temperature: %d\r\n", p[1], temperature * 10 / 16);
        }*/
        
        printf("RF24 p%d 0x%x 0x%x 0x%x 0x%x\r\n", pipe, p[0], p[1], p[2], p[3]);

    }
}

void serial_command(char *cmd)
{
    printf("Command \"%s\"\n", cmd);
#define MATCHSTR(BUF,STR) !strncasecmp(BUF, STR, strlen(STR))
    if (MATCHSTR(cmd, "RADIO")) {
        rf24.printDetails();
    } else if (MATCHSTR(cmd, "PING")) {
        printf("PONG\n");
    }
}

uint8_t therm_letter_from_address(uint8_t addr[8])
{
    for (int i = 0; i < sizeof(thermometer_letter_from_addr)/sizeof(thermometer_letter_from_addr[0]); i++) {
        if (!memcmp(addr, thermometer_letter_from_addr[i].addr, 8)) {
            printf("found letter %c\n", thermometer_letter_from_addr[i].letter);
            return thermometer_letter_from_addr[i].letter;
        }
    }

    printf("Did not identify letter for thermometer addr 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", 
            addr[0],
            addr[1],
            addr[2],
            addr[3],
            addr[4],
            addr[5],
            addr[6],
            addr[7]);
    return 0;
}
void send_temperature()
{
	uint8_t addr[8];
	uint8_t data[12];
	uint8_t present;
	int16_t raw;
	int i = 2;

    uint8_t letter = 0;

    ds.reset_search();
    while (ds.search(addr)) {


        letter = therm_letter_from_address(addr);

        if (!letter) {
            continue;
        }

        ds.reset();
        ds.select(addr);
        ds.write(0x44, 1);
        delay(1000);
        present = ds.reset();
        ds.select(addr);    
        ds.write(0xBE);

        for (int i = 0; i < 9; i++) {
            data[i] = ds.read();
        }

        if (data[8] != OneWire::crc8(data,8)) {
            Serial.println("ERROR: CRC didn't match");
            // Indicate that we read garbage
            send_rf24_cmd(pipe_thermometer, 'T', letter, 0xFF, 0xFF);
            return;
        } else {
            raw = (data[1] << 8) | data[0];
            byte cfg = (data[4] & 0x60);
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
            printf("%c Temperature is %d\n", letter, (int)(100.0*(float)raw/16.0));
        }

        // Simulate receiving an RF24 thermometer command, to blend in with the other, actual RF24 thermometers
        printf("RF24 p%d 0x%x 0x%x 0x%x 0x%x\n", PIPE_THERMOMETER_ID, 'T', letter, (raw>>8) & 0xFF, raw & 0xFF);
    }
}

void loop()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            serial_cmd[serial_cmd_cur] = 0;
            serial_command(serial_cmd);
            serial_cmd_cur = 0;
        } else {
            serial_cmd[serial_cmd_cur++] = c;
        }
    }

#if HAS_RF24
    consume_rf24();
#endif

    if (millis() > send_next_temperature_at) {
        send_temperature();
        send_next_temperature_at += 120000; // 2 minute temperature data
    }
}
