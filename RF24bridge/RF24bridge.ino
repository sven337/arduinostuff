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
const uint64_t pipe_ledlamp  = 0xF0F0F0F0F2LL;
const uint64_t pipe_mailbox  = 0xF0F0F0F0F3LL;
const uint64_t pipe_thermometer  = 0xF0F0F0F0F4LL;

#define PIPE_GAZ_ID 1
#define PIPE_LEDLAMP_ID 2
#define PIPE_MAILBOX_ID 3
#define PIPE_THERMOMETER_ID 4

#define startFrame 0x02
#define endFrame 0x03

RF24 rf24(CE_PIN, CSN_PIN);
SoftwareSerial edfSerial(2, 6); //RX, TX (unused TX)
char teleinfo_buf[255];
int teleinfo_cur = 0;

OneWire ds(DS18B20_PIN);

char serial_cmd[255];
int serial_cmd_cur = 0;

void send_frame()
{
    printf("\r\r\n%s\r\r\n", teleinfo_buf); // printable code for a frame: start and end with \r\r\n
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
            send_frame();
            continue;
        }

        if (inFrame) {
            teleinfo_buf[teleinfo_cur++] = c;
        }
    }
}

int send_rf24_cmd(uint64_t addr, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3)
{
	uint8_t payload[4];
	int ret = -1;
	payload[0] = param0;
	payload[1] = param1;
	payload[2] = param2;
	payload[3] = param3;

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
}

void setup()
{
    printf_begin();
    Serial.begin(57600);
    printf("Hello\n");
    edfSerial.begin(1200); 

	rf24.begin();
	rf24.powerDown();
	rf24.setRetries(15, 15);
	rf24.setChannel(95);
	rf24.setCRCLength(RF24_CRC_16);
	rf24.setPayloadSize(sizeof(unsigned long));
	rf24.setPALevel(RF24_PA_MAX);
	rf24.setDataRate(RF24_250KBPS);
 	rf24.setAutoAck(true);

    rf24.openReadingPipe(PIPE_GAZ_ID, pipe_gaz);
    rf24.openReadingPipe(PIPE_LEDLAMP_ID, pipe_ledlamp);
    rf24.openReadingPipe(PIPE_MAILBOX_ID, pipe_mailbox);
    rf24.openReadingPipe(PIPE_THERMOMETER_ID, pipe_thermometer);
	rf24.startListening();

	rf24.printDetails();

	if ((rf24.getDataRate() != RF24_250KBPS) ||
		(rf24.getCRCLength() != RF24_CRC_16) /*|| 
		(rf24.getChannel() != 95)*/) {
        printf("Failed to initialize radio\n");
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
}

void consume_rf24()
{
    uint8_t data[4];
	uint8_t pipe = 1;

	 while (rf24.available(&pipe)) {
		rf24.read(data, 4);
        printf("RF24 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3]);
    }
}

void serial_command(char *cmd)
{
    printf("Got command \"%s\"\n", cmd);
#define MATCHSTR(BUF,STR) !strncasecmp(BUF, STR, strlen(STR))
    if (MATCHSTR(cmd, "LEDLAMP ")) {
        char *p = cmd + strlen("LEDLAMP ");
        int val = atoi(p);
        int retry = 1;

        if (MATCHSTR(p, "query")) {
            while (retry-- && send_rf24_cmd(pipe_ledlamp, 'Q', 0, 0, 0)) {
                delay(30);
            }
        } else if (MATCHSTR(p, "fade")) {
            while (retry-- && send_rf24_cmd(pipe_ledlamp, 'F', 0, 0, 0)) {
                delay(30);
            }
        } else {
            while (retry-- && send_rf24_cmd(pipe_ledlamp, 'L', val, 0, 0)) {
                delay(30);
            }
        }
    } else if (MATCHSTR(cmd, "RADIO")) {
        rf24.printDetails();
    } else if (MATCHSTR(cmd, "PING")) {
        printf("PONG\n");
    }
}

void loop()
{
    if (edfSerial.available()) {
        consume_teleinfo();
    }

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

    consume_rf24();
}
