#include <SPI.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"

const int CE_PIN = 9;
const int CSN_PIN = 8;
const int DS18B20_PIN = 7;

// XXX pipe 1 must always be used otherwise the others are broken since their
// address is defined as pipe1's except for the least significant byte
#define PIPE_POOL_COVER_ID 1
#define PIPE_LINKY_ID 3
#define PIPE_THERMOMETER_ID 4

struct pipes {
    uint8_t id;
    uint64_t addr;
} pipes[] = {
    {PIPE_POOL_COVER_ID, 0xF0F0F0F0F1LL},
    {PIPE_LINKY_ID, 0xF0F0F0F0F3LL},
    {PIPE_THERMOMETER_ID, 0xF0F0F0F0F4LL},
};

uint64_t pipe_to_addr(uint8_t pipe_id)
{
    for (unsigned int i = 0; i < sizeof(pipes)/sizeof(pipes[0]); i++) {
        if (pipes[i].id == pipe_id) {
            return pipes[i].addr;
        }
    }
    return 0;
}

#define HAS_RF24 1
#define HAS_DS18B20 0

RF24 rf24(CE_PIN, CSN_PIN, 4000000);

OneWire ds(DS18B20_PIN);
uint32_t send_next_temperature_at = 1000;

struct {
    uint8_t addr[8];
    uint8_t letter;
} thermometer_letter_from_addr[] = {
        {{ 0x28, 0xff, 0xbd, 0xd3, 0x90, 0x15, 0x03, 0xbb }, 'R'}, //rochelle (self)
        {{ 0x28, 0x21, 0x5e, 0x79, 0xa2, 0x00, 0x03, 0xc8 }, 'B'}, //bedroom
        {{ 0x28, 0x05, 0x46, 0x79, 0xa2, 0x00, 0x03, 0xe0 }, 'L'}, //living
        {{ 0x28, 0xd0, 0x41, 0x79, 0xa2, 0x00, 0x03, 0x9e }, 'K'}, //kid
        {{ 0x28, 0xff, 0xf0, 0x19, 0x91, 0x15, 0x01, 0x42 }, 'T'}, //test
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
	delayMicroseconds(10000);
	rf24.openWritingPipe(addr);
	rf24.powerUp();
	delayMicroseconds(10000);
	bool ok = rf24.write(&payload[0], 4);

	if (ok) {
		printf("... successful\n");
		ret = 0;
	} else {
		printf("... could not send RF24 cmd\n");
	}
	rf24.startListening();
#endif
	return ret;
}

void setup()
{
    printf_begin();

    // 3.3V Pro mini runs at 8MHz and 115200bps is too fast for it
    // 3.3V Jeenode v6 (leftover from my early days of IoT) runs at 16MHz, so use 115200bps there
    #if (F_CPU == 16000000L)
        Serial.begin(115200);
    #else
        Serial.begin(57600);
    #endif

    printf("RF24bridge starting\n");
#if HAS_RF24
start:
	rf24.begin();
	rf24.powerDown();
	rf24.setRetries(15, 15);
	rf24.setChannel(80);
	rf24.setCRCLength(RF24_CRC_16);
	rf24.setPayloadSize(sizeof(unsigned long));
	rf24.setPALevel(RF24_PA_MAX);
	rf24.setDataRate(RF24_250KBPS);
 	rf24.setAutoAck(true);

    for (int i = 0; i < sizeof(pipes)/sizeof(pipes[0]); i++) {
        rf24.openReadingPipe(pipes[i].id, pipes[i].addr);
    }
	rf24.startListening();

	rf24.printDetails();

	if ((rf24.getDataRate() != RF24_250KBPS) ||
		(rf24.getCRCLength() != RF24_CRC_16)) {
        printf("Failed to initialize radio\n");
        delay(1000);
        goto start;
    }
#endif	
}

void consume_rf24_input()
{
    uint8_t p[4];
  uint8_t pipe = 1;

  while (rf24.available(&pipe)) {
    rf24.read(p, 4);

    printf("RF24 p%d 0x%x 0x%x 0x%x 0x%x\r\n", pipe, p[0], p[1], p[2], p[3]);

  }
}

void serial_command(char *cmd)
{
#define MATCHSTR(BUF,STR) !strncasecmp(BUF, STR, strlen(STR))
    if (MATCHSTR(cmd, "SF24")) {
        // "Send to rF24"
        // Expected format: "SF24 p<pipe_id> <b0> <b1> <b2> <b3>"
        uint8_t pipe_id, b0, b1, b2, b3;
        int n = 0;
        // Skip "SF24" and any whitespace
        char *p = cmd + 5;
        if (*p != 'p') {
            printf("SF24 parse error: expected 'p' after SF24 in %s\n", cmd);
            return;
        }

        p++;
        // Parse pipe_id
        pipe_id = (uint8_t)strtoul(p, &p, 10);

        p++; 

        // Parse 4 bytes
        n = sscanf(p, "%hhx %hhx %hhx %hhx", &b0, &b1, &b2, &b3);
        if (n != 4) {
            printf("SF24 parse error: expected 4 bytes after pipe %d in %s\n", pipe_id, cmd);
            return;
        } 
        uint64_t addr = pipe_to_addr(pipe_id);
        if (!addr) {
            printf("SF24 error: unknown pipe id %d\n", pipe_id);
            return;
        }
        printf("SF24 -> p%d %hhx %hhx %hhx %hhx\n", pipe_id, b0, b1, b2, b3);
        send_rf24_cmd(addr, b0, b1, b2, b3);

    } else if (MATCHSTR(cmd, "RADIO")) {
        rf24.printDetails();
    } else if (MATCHSTR(cmd, "PING")) {
        printf("PONG\n");
    } else {
        printf("Unknown command: %s\n", cmd);
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

void read_ds18b20_sensors()
{
	uint8_t addr[8];
	uint8_t data[12];
	int16_t raw;

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
        ds.reset();
        ds.select(addr);    
        ds.write(0xBE);

        for (int i = 0; i < 9; i++) {
            data[i] = ds.read();
        }

        if (data[8] != OneWire::crc8(data,8)) {
            Serial.println("ERROR: CRC didn't match");
            // Indicate that we read garbage
            send_rf24_cmd(pipe_to_addr(PIPE_THERMOMETER_ID), 'T', letter, 0xFF, 0xFF);
            continue;
        } else {
            raw = (data[1] << 8) | data[0];
            byte cfg = (data[4] & 0x60);
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
            //printf("%c Temperature is %d\n", letter, (int)(100.0*(float)raw/16.0));
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
            if (serial_cmd_cur < sizeof(serial_cmd) - 1) {
                serial_cmd[serial_cmd_cur++] = c;
            }
            // Silently ignore characters if buffer is full
        }
    }

#if HAS_RF24
    consume_rf24_input();
#endif

#if HAS_DS18B20
    if (millis() > send_next_temperature_at) {
        read_ds18b20_sensors();
        send_next_temperature_at += 120000; // 2 minute temperature data
    }
#endif
}
