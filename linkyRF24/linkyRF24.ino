#include <SPI.h>
#include <avr/sleep.h>
#include <JeeLib.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"
#include <avr/sleep.h>

const int CE_PIN = 9;
const int CSN_PIN = 8;
const int DS18B20_PIN = 7;
const int WATER_PULSE_PIN = 6;
const int LED_YELLOW = 5;
const int LED_RED = 4;
const int GAS_PULSE_PIN = 3;
const int LINKY_RX = 2;

#define startFrame 0x02
#define endFrame 0x03

const uint64_t pipe_address = 0xF0F0F0F0F3LL;
RF24 rf24(CE_PIN, CSN_PIN);
OneWire ds(DS18B20_PIN);

// Pin change interrupts are not compatible with SoftwareSerial :(
struct {
    bool armed;
    uint32_t holdoff;
    int pin;
    char radiochar;
} watergas[] = {
        { true, 0, WATER_PULSE_PIN, 'E' },
        { true, 0, GAS_PULSE_PIN, 'G' },
};

SoftwareSerial edfSerial(LINKY_RX, LED_YELLOW); //RX, TX (unused TX)
char teleinfo_buf[255];
int teleinfo_cur = 0;

int8_t frame_bigskip_counter; // how many more frames to skip (for unimportant fields)
int8_t frame_smallskip_counter; // how many more frames to skip (for important fields)
uint32_t next_report_battery_at;
static unsigned long send_next_temperature_at = 0; // Time for next temperature send

// Thermometer address to identification letter mapping
const struct {
    uint8_t addr[8];
    uint8_t letter;
} thermometer_letter_from_addr[] = {
    {{ 0x28, 0xFF, 0xBD, 0xD3, 0x90, 0x15, 0x03, 0xBB }, 'L'}, // Linky thermometer
};

const uint8_t num_known_thermometers = sizeof(thermometer_letter_from_addr) / sizeof(thermometer_letter_from_addr[0]);

// Function to get identification letter for a given address
uint8_t get_thermometer_letter(uint8_t addr[8]) {
    for (uint8_t i = 0; i < num_known_thermometers; i++) {
        bool match = true;
        for (uint8_t j = 0; j < 8; j++) {
            if (thermometer_letter_from_addr[i].addr[j] != addr[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            return thermometer_letter_from_addr[i].letter;
        }
    }
    return 0; // Unknown address
}

// Function to print address in copy-pasteable format
void print_unknown_address(uint8_t addr[8]) {
    Serial.print(F("Unknown thermometer address: {{ 0x"));
    for (uint8_t i = 0; i < 8; i++) {
        if (addr[i] < 0x10) Serial.print(F("0"));
        Serial.print(addr[i], HEX);
        if (i < 7) Serial.print(F(", 0x"));
    }
    Serial.println(F(" }, 'X'}, //unknown"));
}

#define BIGSKIP_INTERVAL 30 // count 30 full frames (approx 90sec)
#define SMALLSKIP_INTERVAL 3  // approx 9 seconds
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

// Verify ERDF Linky checksum
// Format: LABEL DATA CHECKSUM
// Checksum is calculated over: LABEL + SP + DATA (including the space between label and data)
// Algorithm: sum all ASCII codes, keep 6 LSBs (& 0x3F), add 0x20
bool verify_checksum(const char *line) {
    int len = strlen(line);
    
    // Remove trailing \n or \r if present
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
        len--;
    }
    if (len < 3) return false; // Too short to be valid
    
    // The checksum has to be the last character
    int checksum_pos = len - 1;
    // ... and have a space before it
    if (line[checksum_pos - 1] != ' ')
        return false;

    // Extract checksum from buffer
    char received_checksum = line[checksum_pos];
    
    // Calculate expected checksum from start to space before checksum (exclusive)
    // This includes: LABEL + SP + DATA
    uint8_t sum = 0;
    for (int i = 0; i < checksum_pos - 1; i++) {
        sum += line[i];
    }
    
    // Apply checksum algorithm: keep 6 LSBs and add 0x20
    char calculated_checksum = (sum & 0x3F) + 0x20;
    
    return (calculated_checksum == received_checksum);
}

void send_line()
{
    if (!verify_checksum(teleinfo_buf)) {
        Serial.print("linky checksum error: ");
        Serial.println(teleinfo_buf);
        return;
    }
    
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
            { "BBRHCJR", 'r', 0, 0}, //000000000 -
            { "BBRHPJR", 'R', 0, 0}, //000000000 :
            { "HCHP",    'H', 0, 0}, //000000000 
            { "HCHC",    'C', 0, 0}, //000000000 
            { "BASE",    'S', 0, 0}, //000000000 
            { "PTEC",    'J', 1, 0}, //HPJB P
//          { "DEMAIN",   0, 0, 0}, //---- "
            { "IINST",   'I', 0, 1}, //003 Z
            { "ADPS",    'A', 0, 1},
//          { "IMAX",     0 , 0, 0}, //090 H
            { "PAPP",    'P', 1, 1}, //00000 !
/*          { "HHPHC",    0 , 0, 0}, //A , 
            { "MOTDETAT", 0 , 0, 0}, //000000 B */
    };

#define MATCH(X) !memcmp(teleinfo_buf, X, strlen(X))

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

        if (MATCH("BBRH") || MATCH("HCHP") || MATCH("HCHC") || MATCH("BASE")) {
            // Cannot fit the whole index in 24 bits, so trim the 6 LSBs
            uint32_t val = strtol(teleinfo_buf + pos, NULL, 10);
            val = val >> 6;
            data[1] = (val >> 16) & 0xFF;
            data[2] = (val >>  8) & 0xFF;
            data[3] = val & 0xFF;
        }  else {
            memcpy(&data[1], teleinfo_buf + pos, 3);
        }

        // Send data
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
    }
}

ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

int radio_send(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
	uint8_t payload[4] = { p0, p1, p2, p3 };
	digitalWrite(LED_YELLOW, 1);
    rf24.powerUp();
	bool ok = rf24.write(payload, 4);
	if (ok) 
		Serial.println("send ok");
	else
		Serial.println("send KO");
	
    printf("sending %c %c %c %c\n", p0, p1, p2, p3);
	digitalWrite(LED_YELLOW, 0);
    
    rf24.powerDown();

	if (!ok) {
		digitalWrite(LED_RED, 1);
		return -1;
	}
	digitalWrite(LED_RED, 0);
	return 0;
}

void setup(){
	printf_begin();
	Serial.begin(115200);
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
        while (1) {
            int red = 0;
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

    radio_send('D', 0, 0, 0);
	rf24.powerDown();

    pinMode(WATER_PULSE_PIN, INPUT_PULLUP);
    pinMode(GAS_PULSE_PIN, INPUT_PULLUP);
    
}

void loop() 
{
    if (edfSerial.available()) {
        consume_teleinfo();
    }

    for (int i = 0; i < sizeof(watergas)/sizeof(watergas[0]); i++) {
       /* if (millis() > watergas[i].holdoff)*/ {
            // Rearm for next pulse
            if (digitalRead(watergas[i].pin)) {
                watergas[i].armed = true;
            } else {
                // If 0 (pulse in progress)...
                if (watergas[i].armed) {
                    // ... send it if armed, then disarm until pin value changes
                    radio_send(watergas[i].radiochar, 0, 0, 0);
                    watergas[i].armed = false;
                }
                watergas[i].holdoff = millis() + 200;
                // XXX overflow hazard
            }
        }
    }

    // Send temperature
    if (millis() >= send_next_temperature_at) {
        send_next_temperature_at = millis() + 15 * 60 * 1000LL; // 15 minutes

        // Send temperature data from all thermometers
        uint8_t addr[8];
        uint8_t thermometer_count = 0;
        
        // Search for all DS18B20 devices
        ds.reset_search();
        while (ds.search(addr)) {
            thermometer_count++;
            
            // Get identification letter for this address
            uint8_t identification_letter = get_thermometer_letter(addr);
            if (identification_letter == 0) {
                // Unknown thermometer - print address for copy-paste
                print_unknown_address(addr);
                continue; // Skip unknown thermometers
            }

            // Read temperature from this thermometer
            ds.reset();
            ds.select(addr);
            ds.write(0x44, 1); // Start temperature conversion
            delay(1000); // Wait for conversion
            
            uint8_t present = ds.reset();
            ds.select(addr);    
            ds.write(0xBE); // Read scratchpad

            uint8_t data[9];
            for (int i = 0; i < 9; i++) {
                data[i] = ds.read();
            }

            // Check CRC
            if (data[8] != OneWire::crc8(data, 8)) {
                Serial.print(F("ERROR: CRC mismatch for thermometer "));
                Serial.println((char)identification_letter);
                // Indicate that we read garbage
                radio_send('T', identification_letter, 0xFF, 0xFF);
                continue;
            }
            
            // Convert temperature
            int16_t raw = (data[1] << 8) | data[0];
            byte cfg = (data[4] & 0x60);
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
            
            float temperature_c = (float)raw / 16.0;
            printf("Temperature %c is %d\n", identification_letter, (int)(100.0 * temperature_c));

            // Send temperature
            radio_send('T', identification_letter, (raw >> 8) & 0xFF, raw & 0xFF);
        }
        
        if (thermometer_count == 0) {
            Serial.println("No DS18B20 thermometers found");
        }
    }

    /*set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();*/
}



