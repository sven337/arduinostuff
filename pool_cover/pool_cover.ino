#include <SPI.h>
#include <JeeLib.h>
#include <OneWire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <INA226.h>
#include "printf.h" 
#include "nRF24L01.h"
#include "RF24.h"

#define HAS_RF24 1

/*
 * Complete Pin Assignment for Arduino Pro Mini:
 * 
 * Digital Pins:
 *   0  - RX (Serial) - Reserved for programming/debugging
 *   1  - TX (Serial) - Reserved for programming/debugging  
 *   2  - Encoder CLK (interrupt capable)
 *   3  - RF24 IRQ (interrupt capable) - for radio packet reception
 *   4  - Encoder DT (direction)
 *   5  - Motor PWM A (PWM)
 *   6  - Motor PWM B (PWM)
 *   7  - DS18B20 temperature sensor
 *   8  - RF24 CSN
 *   9  - RF24 CE (PWM)
 *   10 - INA226 Alert pin (Pin Change Interrupt capable)
 *   11 - RF24 MOSI (PWM, SPI)
 *   12 - RF24 MISO (SPI)
 *   13 - RF24 SCK (SPI) + onboard LED
 * 
 * Analog Pins:
 *   A0 - Motor Enable (both BTN7960B enables connected together)
 *   A1 - Available
 *   A2 - Available
 *   A3 - Available
 *   A4 - I2C SDA (INA226)
 *   A5 - I2C SCL (INA226)
 *   A6 - Motor Current Sense (analog only)
 *   A7 - Available
 */

const int SCL_PIN = A5;        // I2C for INA226
const int SDA_PIN = A4;        // I2C for INA226
const int INA226_ALERT_PIN = 10; // INA226 alert pin (Pin Change Interrupt)

const int CE_PIN = 9;          // RF24 CE
const int CSN_PIN = 8;         // RF24 CSN
const int RF24_IRQ_PIN = 3;    // RF24 IRQ pin for external interrupts
const int DS18B20_PIN = 7;     // DS18B20 temperature sensor

// H-bridge motor control pins
const int MOTOR_PWM_A_PIN = 5;    // PWM pin for motor A (H-bridge left side)
const int MOTOR_PWM_B_PIN = 6;    // PWM pin for motor B (H-bridge right side) 
const int MOTOR_ENABLE_PIN = A0;  // Enable pin for both BTN7960B (connected together)
const int MOTOR_CURRENT_PIN = A6; // Motor current sense (analog only)

// KY-040 Encoder pins
const int ENCODER_CLK_PIN = 2;  // Interrupt pin (CLK)
const int ENCODER_DT_PIN = 4;   // Direction pin (DT)


OneWire ds(DS18B20_PIN);
const uint8_t INA226_I2C_ADDRESS = 0x44;
INA226 ina226(INA226_I2C_ADDRESS); 

// Overcurrent protection variables
static unsigned long overcurrent_start_time = 0;
static bool overcurrent_detected = false;
static const float OVERCURRENT_THRESHOLD = 6.0; // 6 Amperes
static const unsigned long OVERCURRENT_TIMEOUT = 15000; // 15 seconds

// INA226 alert handling
volatile bool ina226_alert_triggered = false;

#if HAS_RF24
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address_cover = 0xF0F0F0F0F1LL;
const uint64_t pipe_address_temperature = 0xF0F0F0F0F4LL;  
#define PIPE_POOL_COVER 1
#define PIPE_TEMPERATURE 4
#endif

static unsigned long motor_duration_up = 9.5 * 60 * 1000UL; // 10 minutes for up
static unsigned long motor_duration_down = 8.5 * 60 * 1000UL; // 9 minutes for down
static unsigned long motor_stop_at = 0;
static bool motor_running = false;
static char motor_direction = 'U';
static unsigned long send_next_temperature_at = 0; // Time for next temperature send
static unsigned long send_next_status_at = 0; // Time for next status send

// Reboot mechanism to avoid millis() overflow issues
const unsigned long REBOOT_AT_MILLIS = 4200000000UL; // ~48.6 days

// RF24 interrupt handling
volatile bool radio_packet_received = false;

// Encoder and position tracking variables
const long DEFAULT_TRAVEL_DISTANCE = 30 * 12; // 30 steps/turn * 12 turns
volatile long encoder_position = 0;  // Current encoder position
static long encoder_target_position = 0;  // Target position for current movement
static long encoder_top_position = -DEFAULT_TRAVEL_DISTANCE;     // Marked top position
static long encoder_bottom_position = DEFAULT_TRAVEL_DISTANCE; // Marked bottom position
static bool has_marked_top = false;
static bool has_marked_bottom = false;


int init_failed = 0;
bool ina226_initialized = false;

// Thermometer address to identification letter mapping
const struct {
    uint8_t addr[8];
    uint8_t letter;
} thermometer_letter_from_addr[] = {
    /*{{ 0x28, 0xff, 0xf0, 0x19, 0x91, 0x15, 0x01, 0x42 }, 'T'}, //test
	{{ 0x28, 0xFF, 0x9A, 0xEA, 0x90, 0x15, 0x01, 0x75 }, 'T'}, //test too*/
	 {{ 0x28, 0x84, 0xCC, 0xDC, 0x04, 0x00, 0x00, 0xA6 }, 'T'}, //test
	{{ 0x28, 0xD7, 0xC4, 0xD9, 0x04, 0x00, 0x00, 0x6E }, 'P'}, // swimming pool (water)

};

const uint8_t num_known_thermometers = sizeof(thermometer_letter_from_addr) / sizeof(thermometer_letter_from_addr[0]);
		

/* My INA226 chip is off by a lot (possibly counterfeit?) https://github.com/RobTillaart/INA226/issues/30
	* Measured -> Actual
	* 17.062  -> 16.20
	* 17.5    -> 16.61  
	* 13.190  -> 12.51
	* 12.137  -> 11.51
	* 11.081  -> 10.50
	* 10.025  -> 9.510
	* Correction factor: 1.0542
	*/
const float INA226_VOLTAGE_CORRECTION = 1.0542f;

// INA226 hardware constants
const float SHUNT_RESISTANCE_OHMS = 0.012f;       // 12 milliohm shunt (R012)
const float INA226_SHUNT_VOLTAGE_LSB = 0.0000025f; // 2.5µV per LSB for shunt voltage register

// Encoder interrupt handler
void encoder_interrupt() {
	bool encoder_direction_cw;
    static unsigned long last_interrupt_time = 0;
    
    unsigned long interrupt_time = millis();
    
    // Debounce
    if (interrupt_time - last_interrupt_time < 20) {
        return;
    }

	// Read both pins to determine direction
    bool clk_state = digitalRead(ENCODER_CLK_PIN);
    bool dt_state = digitalRead(ENCODER_DT_PIN);
    
    // If CLK and DT are equal, we're moving clockwise
    encoder_direction_cw = (clk_state == dt_state);
    
    if (encoder_direction_cw) {
        encoder_position++;
    } else {
        encoder_position--;
    }
    
	last_interrupt_time = millis();
}

// RF24 interrupt handler
void rf24_interrupt() {
    radio_packet_received = true;
}

// INA226 interrupt handler
void ina226_interrupt() {
    // INA226 alert triggered - set flag for main loop processing
    ina226_alert_triggered = true;
}

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


ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

// Software reboot function
void software_reboot() {
	Serial.println(F("Rebooting to prevent millis() overflow..."));
	Serial.flush();
	
	// Disable interrupts
	cli();
	
	// Set watchdog timer to shortest timeout (16ms) and enable system reset mode
	wdt_enable(WDTO_15MS);
	
	// Wait for watchdog to trigger reset
	while(1) {}
}

void stop_motor()
{
	digitalWrite(MOTOR_ENABLE_PIN, LOW);
	analogWrite(MOTOR_PWM_A_PIN, 0);
	analogWrite(MOTOR_PWM_B_PIN, 0);
	motor_running = false;
	
	// Reset overcurrent protection
	overcurrent_detected = false;
	
	// Put INA226 to sleep when motor stops
	ina226_sleep();
	
	send_cover_status();
}

void start_motor(char direction)
{
	// Stop motor first
	analogWrite(MOTOR_PWM_A_PIN, 0);
	analogWrite(MOTOR_PWM_B_PIN, 0);
	digitalWrite(MOTOR_ENABLE_PIN, LOW);
	delay(100);
	
	// Enable both BTN7960B half-bridges
	digitalWrite(MOTOR_ENABLE_PIN, HIGH);
	
	// Set direction and start motor at full speed
	if (direction == 'U') {
		// Up direction: Left side active, right side off
		analogWrite(MOTOR_PWM_A_PIN, 255);
		analogWrite(MOTOR_PWM_B_PIN, 0);
	} else { // direction == 'D'
		// Down direction: Left side off, right side active
		analogWrite(MOTOR_PWM_A_PIN, 0);
		analogWrite(MOTOR_PWM_B_PIN, 255);
	}
	
	motor_running = true;
	motor_direction = direction;
	// Use different durations based on direction
	unsigned long duration = (direction == 'U') ? motor_duration_up : motor_duration_down;
	motor_stop_at = millis() + duration;
	
	// Wake up INA226 for current monitoring
	ina226_wake();
	
	// Set target encoder position based on direction and limits
	long current_pos = encoder_position;
	
	if (direction == 'U') {
		// Moving up - target is top position or current + default travel
		if (has_marked_top) {
			encoder_target_position = encoder_top_position;
		} else {
			encoder_target_position = current_pos - DEFAULT_TRAVEL_DISTANCE;
		}
	} else { // direction == 'D'
		// Moving down - target is bottom position or current + default travel
		if (has_marked_bottom) {
			encoder_target_position = encoder_bottom_position;
		} else {
			encoder_target_position = current_pos + DEFAULT_TRAVEL_DISTANCE;
		}
	}
	
	send_cover_status();
	// Start 15-second cycle for frequent updates while motor running
	send_next_status_at = millis() + 5000LL;
}


int radio_send(uint8_t pipe_id, uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
#if HAS_RF24
	uint8_t payload[4] = { p0, p1, p2, p3 };
	
	// Must stop listening before changing writing pipe
	radio.stopListening();
	
	// Select the appropriate writing pipe
	if (pipe_id == PIPE_POOL_COVER) {
		radio.openWritingPipe(pipe_address_cover);
	} else {
		radio.openWritingPipe(pipe_address_temperature);
	}
	
	delayMicroseconds(5000);
	bool ok = radio.write(payload, 4);
	Serial.print(F("radio_send p"));
	Serial.print(pipe_id);
	Serial.print(F("["));
	Serial.print((char)p0);
	Serial.print(F(" 0x"));
	if (p1 < 16) Serial.print(F("0"));
	Serial.print(p1, HEX);
	Serial.print(F(" 0x"));
	if (p2 < 16) Serial.print(F("0"));
	Serial.print(p2, HEX);
	Serial.print(F(" 0x"));
	if (p3 < 16) Serial.print(F("0"));
	Serial.print(p3, HEX);
	Serial.print(F("] -> "));
	Serial.println(ok ? F("OK") : F("KO"));
	
	// Resume listening after sending
	radio.startListening();

	if (!ok) {
		return -1;
	}
#endif
	return 0;
}

// Convert analog reading to millivolts
uint16_t analog_to_millivolts(uint16_t analog_reading)
{
	// Arduino ADC: 0-1023 for 0-5V (or 0-3.3V depending on reference)
	// Assuming 3.3V reference: mV = analog_reading * 3300 / 1024
	return (uint32_t)analog_reading * 3300 / 1024;
}

// Convert BTN7960B IS pin analog reading to motor current in milliamps
// Using 10kΩ || 6.8kΩ = 4.05kΩ external resistor for 7A full-scale with 3.3V ADC
// With 4.05kΩ external resistor and kILIS = 8500:
// VIS = (IL / 8500) × 4050, so IL = VIS × 8500 / 4050 = VIS × 2.099 A/V
uint16_t btn7960_analog_to_motor_current_ma(uint16_t analog_reading)
{
	// Convert ADC reading to voltage (mV)
	uint16_t voltage_mv = analog_to_millivolts(analog_reading);
	
	// Convert voltage to current: IL = VIS × (8500/4050) A/V = VIS × 2.099 A/V
	// IL(mA) = VIS(mV) × 2.099 = VIS(mV) × 2099 / 1000
	uint32_t current_ma = (uint32_t)voltage_mv * 2099 / 1000;
	
	// With 4.05kΩ resistor (10kΩ || 6.8kΩ): 7A → 3.33V, optimal ADC usage
	// ADC resolution: ~7.3mA per step
	if (current_ma > 7500) current_ma = 7500;  // Safety margin
	
	return (uint16_t)current_ma;
}

// Read motor current from BTN7960B IS pin
uint16_t read_motor_current_ma()
{
	uint16_t analog_reading = analogRead(MOTOR_CURRENT_PIN);
	return btn7960_analog_to_motor_current_ma(analog_reading);
}

// Pool cover RF24 protocol functions
void send_battery_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_POOL_COVER, 'V', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
}

void send_solar_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_POOL_COVER, 'S', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
}

// Sends battery power as signed 24-bit milliwatts (two's complement), little-endian in 3 data bytes.
// This carries full mW resolution and increases dynamic range beyond int16. Receiver must sign-extend.
void send_battery_power(int32_t power_mw)
{
    // Clamp to signed 24-bit range
    const int32_t MAX_24BIT_SIGNED = (1L << 23) - 1;
    const int32_t MIN_24BIT_SIGNED = -(1L << 23);
    
    if (power_mw > MAX_24BIT_SIGNED) power_mw = MAX_24BIT_SIGNED;
    if (power_mw < MIN_24BIT_SIGNED) power_mw = MIN_24BIT_SIGNED;

    // Convert to two's complement representation in the lower 24 bits of a uint32_t
    uint32_t u24 = (uint32_t)power_mw & 0x00FFFFFF;
    
    // Extract bytes in little-endian order
    uint8_t b0 = (uint8_t)(u24 & 0xFF);
    uint8_t b1 = (uint8_t)((u24 >> 8) & 0xFF);
    uint8_t b2 = (uint8_t)((u24 >> 16) & 0xFF);
    
    radio_send(PIPE_POOL_COVER, 'P', b0, b1, b2);
}

void send_battery_current(int16_t current_ma)
{
	radio_send(PIPE_POOL_COVER, 'I', current_ma & 0xFF, (current_ma >> 8) & 0xFF, 0);
}

void send_motor_current(uint16_t current_ma)
{
	radio_send(PIPE_POOL_COVER, 'M', current_ma & 0xFF, (current_ma >> 8) & 0xFF, 0);
}

void send_encoder_position(int32_t position)
{
	radio_send(PIPE_POOL_COVER, 'E', position & 0xFF, (position >> 8) & 0xFF, (position >> 16) & 0xFF);
}

void send_cover_status()
{
	// 'Q' for status, motor_direction is 'U'/'D'/'S'/'b' (booting)
	radio_send(PIPE_POOL_COVER, 'Q', motor_running ? motor_direction : 'S', 0, 0); 
	
	// If motor is running, also send current encoder position
	if (motor_running) {
		send_encoder_position((uint32_t)encoder_position);
	}
}

void process_cover_command(uint8_t cmd, uint8_t param1, uint8_t param2, uint8_t param3)
{
	Serial.print(F("Cover command: "));
	Serial.print((char)cmd);
	Serial.print(F(" "));
	Serial.print((char)param1);
	Serial.println();
	
	if (cmd == 'C') { // Command
		switch (param1) {
			case 'U': // Up
				start_motor('U');
				break;
			case 'D': // Down
				start_motor('D');
				break;
			case 'S': // Stop
				stop_motor();
				break;
			case 'M': // Mark travel limit
				if (param2 == 'T') {
					// Mark current position as top
					encoder_top_position = encoder_position;
					has_marked_top = true;
					Serial.print(F("Top marked at encoder position: "));
					Serial.println(encoder_top_position);
				} else if (param2 == 'D') {
					// Mark current position as bottom
					encoder_bottom_position = encoder_position;
					has_marked_bottom = true;
					Serial.print(F("Bottom marked at encoder position: "));
					Serial.println(encoder_bottom_position);
				}
				break;
			case 'Q': // Query status
				send_cover_status();
				break;
		}
	} else if (cmd == 'D') { // Set motor duration
		unsigned long duration = param1 | (param2 << 8) | (param3 << 16); // duration in seconds
		// Convert seconds to milliseconds
		duration = duration * 1000UL;
		// Sanity check: limit to 10 minutes
		if (duration > 600000UL) {
			duration = 600000UL;
		}
		// Set both up and down durations to the same value for remote commands
		motor_duration_up = duration;
		motor_duration_down = duration;
	}
}

void check_radio_messages()
{
#if HAS_RF24
	uint8_t pipe_num;
	bool packet_processed = false;
	
	while (radio.available(&pipe_num)) {
		uint8_t payload[4];
		radio.read(payload, 4);
		packet_processed = true;
		
		
		if (pipe_num == PIPE_POOL_COVER) {
			process_cover_command(payload[0], payload[1], payload[2], payload[3]);
		}
	}
	
	// Clear RF24 interrupt flags after processing all packets
	if (packet_processed) {
		bool tx_ok, tx_fail, rx_ready;
		radio.whatHappened(tx_ok, tx_fail, rx_ready); // This clears the interrupt flags
	}
#endif
}

// INA226 identification functions
uint16_t read_ina226_register(uint8_t reg_addr) {
	Wire.beginTransmission(INA226_I2C_ADDRESS);
	Wire.write(reg_addr);
	if (Wire.endTransmission() != 0) {
		return 0xFFFF; // Error indicator
	}
	
	Wire.requestFrom(INA226_I2C_ADDRESS, 2);
	if (Wire.available() != 2) {
		return 0xFFFF; // Error indicator
	}
	
	uint16_t value = Wire.read() << 8; // High byte first
	value |= Wire.read(); // Low byte
	return value;
}

void check_ina226_identification() {
	Serial.println(F("Reading INA226 identification registers..."));
	
	// Read Manufacturer ID register (0xFE)
	uint16_t manufacturer_id = read_ina226_register(0xFE);
	Serial.print(F("Manufacturer ID (0xFE): 0x"));
	Serial.print(manufacturer_id, HEX);
	
	if (manufacturer_id == 0x5449) {
		Serial.println(F(" (Texas Instruments)"));
	} else if (manufacturer_id == 0xFFFF) {
		Serial.println(F(" - read error"));
	} else {
		Serial.println(F(" - unexpected value, expected 0x5449"));
	}
	
	// Read Die ID register (0xFF)
	uint16_t die_id = read_ina226_register(0xFF);
	Serial.print(F("Die ID (0xFF): 0x"));
	Serial.print(die_id, HEX);
	
	if (die_id == 0xFFFF) {
		Serial.println(F(" - read error"));
	} else {
		// Extract device ID (bits 15-4) and die revision (bits 3-0)
		uint16_t device_id = (die_id >> 4) & 0x0FFF;
		uint8_t die_revision = die_id & 0x0F;
		
		Serial.print(F(" (Device ID: 0x"));
		Serial.print(device_id, HEX);
		Serial.print(F(", Die Rev: 0x"));
		Serial.print(die_revision, HEX);
		
		if (device_id == 0x226) {
			Serial.println(F(")"));
		} else {
			Serial.println(F(" - expected device ID 0x226)"));
		}
	}
	
	Serial.println();
}

// INA226 power management functions
void ina226_sleep() {
	if (ina226_initialized && ina226.isCalibrated()) {
		ina226.shutDown();
	}
}

void ina226_wake() {
	if (ina226_initialized && ina226.isCalibrated()) {
		ina226.setModeShuntBusContinuous();
		// Re-configure alert after wake up
		float alert_current = 6.0;  // Alert at 6.0A
		uint16_t alert_limit = (alert_current * SHUNT_RESISTANCE_OHMS) / INA226_SHUNT_VOLTAGE_LSB;  // Convert to register counts
		ina226.setAlertLimit(alert_limit);
		ina226.setAlertRegister(INA226_SHUNT_OVER_VOLTAGE);
	}
}

void setup(){
	printf_begin();
	Serial.begin(115200);
	Serial.println(F("Pool cover controller starting..."));  

	// H-bridge motor control pins
	pinMode(MOTOR_PWM_A_PIN, OUTPUT);
	pinMode(MOTOR_PWM_B_PIN, OUTPUT);
	pinMode(MOTOR_ENABLE_PIN, OUTPUT);
	// Note: A6 is analog-only, no pinMode needed for current sense

	// INA226 alert pin
	pinMode(INA226_ALERT_PIN, INPUT_PULLUP);
	// Attach Pin Change Interrupt to INA226 alert pin (pin 10)
	attachPCINT(digitalPinToPCINT(INA226_ALERT_PIN), ina226_interrupt, FALLING);

	// Encoder pins
	pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
	pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
	
	// Attach interrupt to encoder CLK pin
	attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoder_interrupt, FALLING);

	// RF24 IRQ pin
	pinMode(RF24_IRQ_PIN, INPUT_PULLUP); 
	attachInterrupt(digitalPinToInterrupt(RF24_IRQ_PIN), rf24_interrupt, FALLING);

#if HAS_RF24
	// Radio init
	radio.begin();
	radio.powerDown();
	radio.setRetries(15, 15);
	radio.setChannel(80);
	radio.setCRCLength(RF24_CRC_16);
	radio.setPayloadSize(4);
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
 	radio.setAutoAck(true);
	
	// Setup both pipes for reading and writing
	radio.openWritingPipe(pipe_address_temperature);
	radio.openReadingPipe(PIPE_TEMPERATURE, pipe_address_temperature);
	radio.openReadingPipe(PIPE_POOL_COVER, pipe_address_cover);
	
	// Enable interrupt ONLY on data received (RX_DR)
	// maskIRQ(tx_ds, tx_fail, rx_ready) - 1=mask(disable), 0=enable
	radio.maskIRQ(1, 1, 0); // Mask TX_DS and MAX_RT, enable RX_DR only
	
	radio.startListening();

	radio.printDetails();
	
	if ((radio.getDataRate() != RF24_250KBPS) ||
		(radio.getCRCLength() != RF24_CRC_16) /*|| 
		(radio.getChannel() != 95)*/) {
		// failed to initialize radio
		init_failed = 1;
	}
#endif

	// Initialize I2C for INA226
	Wire.begin();
	Serial.println(F("I2C initialized"));
	
	// Initialize INA226
	if (!ina226.begin()) {
		Serial.print(F("ERROR: Failed to initialize INA226 at address 0x"));
		Serial.println(INA226_I2C_ADDRESS, HEX);
		Serial.println(F("Check wiring and connections"));
		init_failed = 1;
	} else {
		Serial.print(F("INA226 initialized successfully at 0x"));
		Serial.println(INA226_I2C_ADDRESS, HEX);
		ina226_initialized = true;
		
		// Check INA226 identification to verify authenticity
		check_ina226_identification();
	}
	
	// Configure INA226 if it was successfully initialized
	if (ina226_initialized) {
		Serial.println(F("Configuring INA226..."));
		
		// Calculate max current based on INA226 library constraint
		// Library checks: maxCurrent * shunt <= 0.08190V (81.90mV)
		float max_current = 0.08190 / SHUNT_RESISTANCE_OHMS;  // Maximum current the library will accept
		
		Serial.print(F("Calibrating with max current: "));
		Serial.print(max_current);
		Serial.print(F("A with "));
		Serial.print(SHUNT_RESISTANCE_OHMS);
		Serial.print(F(" ohm shunt... "));
		
		int cal_result = ina226.setMaxCurrentShunt(max_current, SHUNT_RESISTANCE_OHMS, true);
		if (cal_result == 0) {
			Serial.println(F("SUCCESS!"));
			
			Serial.print(F("INA226 calibrated successfully. Current LSB: "));
			Serial.print(ina226.getCurrentLSB_mA());
			Serial.println(F(" mA"));
			Serial.print(F("IMPORTANT: Current measurement range up to "));
			Serial.print(max_current);
			Serial.print(F("A with "));
			Serial.print(SHUNT_RESISTANCE_OHMS);
			Serial.println(F("Ω shunt!"));
			
			// Set conversion time for both shunt and bus voltage (default is fine)
			// Enable continuous shunt and bus voltage monitoring
			ina226.setModeShuntBusContinuous();
			
			// Configure alert for overcurrent detection (6.0A threshold)
			// Calculate alert limit: I_alert * R_shunt / LSB_shunt_voltage
			// INA226 shunt voltage register LSB is 2.5µV
			float alert_current = 6.0;  // Alert at 6.0A
			uint16_t alert_limit = (alert_current * SHUNT_RESISTANCE_OHMS) / INA226_SHUNT_VOLTAGE_LSB;  // Convert to register counts
			if (!ina226.setAlertLimit(alert_limit)) {
				Serial.println(F("ERROR: Failed to set INA226 alert limit"));
			}
			
			// Configure alert register for shunt overvoltage (overcurrent)
			if (!ina226.setAlertRegister(INA226_SHUNT_OVER_VOLTAGE)) {
				Serial.println(F("ERROR: Failed to configure INA226 alert register"));
			} else {
				Serial.print(F("INA226 alert configured for overcurrent at "));
				Serial.print(OVERCURRENT_THRESHOLD);
				Serial.println(F("A"));
			}
		} else {
			Serial.print(F("FAILED (0x"));
			Serial.print(cal_result, HEX);
			Serial.println(F(")"));
			Serial.println(F("ERROR: INA226 calibration failed"));
			ina226_initialized = false;
			init_failed = 1;
		}
	}

	Serial.println(F("Pool cover controller ready"));

	radio_send(PIPE_POOL_COVER, 'Q', 'b', 0, 0); //"booting"
	stop_motor();
}

void loop() 
{
	if (radio_packet_received) {
		radio_packet_received = false; // Clear flag
		check_radio_messages();
	}

	// Drive motor
	if (motor_running) {
		check_radio_messages(); // Not needed most of the time, but I notice that when the motor is running, sometimes radio messages are missed. Let's see if this helps.

		bool must_stop = false;
		
		// Check for INA226 alert (overcurrent)
		if (ina226_initialized && ina226_alert_triggered) {
			ina226_alert_triggered = false; // Clear flag
			
			// Read current to confirm and get exact value
			float current_a = ina226.getCurrent();
			
			if (current_a >= OVERCURRENT_THRESHOLD) {
				if (!overcurrent_detected) {
					// Start overcurrent timer
					overcurrent_detected = true;
					overcurrent_start_time = millis();
					Serial.print(F("Overcurrent alert triggered: "));
					Serial.print(current_a);
					Serial.println(F("A - starting timer"));
				}
			}
			
			// Clear the alert flag in INA226 by reading the alert register
			ina226.getAlertFlag();

			send_next_status_at = millis(); // Send update immediately
		}
		
		// Check overcurrent timer if overcurrent was detected
		if (overcurrent_detected) {
			float current_a = ina226.getCurrent();
			
			if (current_a >= OVERCURRENT_THRESHOLD) {
				// Still overcurrent - check timeout
				if (millis() - overcurrent_start_time >= OVERCURRENT_TIMEOUT) {
					Serial.print(F("Motor stopped: overcurrent protection ("));
					Serial.print(current_a);
					Serial.print(F("A for "));
					Serial.print((millis() - overcurrent_start_time) / 1000.0);
					Serial.println(F(" seconds)"));
					must_stop = true;
				}
			} else {
				// Current dropped below threshold - reset
				Serial.print(F("Overcurrent cleared: "));
				Serial.print(current_a);
				Serial.println(F("A"));
				overcurrent_detected = false;
			}
		}
		
		// Check if time limit reached
		if (millis() >= motor_stop_at) {
			Serial.println(F("Motor stopped: time limit reached"));
			must_stop = true;
		}
		
		if (motor_direction == 'U') {
			// Moving up - stop if we've reached or passed the target (going negative)
			if (encoder_position <= encoder_target_position) {
				Serial.print(F("Motor stopped: up position limit reached at "));
				Serial.println(encoder_position);
				must_stop = true;
			}
		} else { // motor_direction == 'D'
			// Moving down - stop if we've reached or passed the target (going positive)
			if (encoder_position >= encoder_target_position) {
				Serial.print(F("Motor stopped: down position limit reached at "));
				Serial.println(encoder_position);
				must_stop = true;
			}
		}
		
		if (must_stop) {
			stop_motor();
		}
	}
		
	// Send status
	if (millis() >= send_next_status_at) {
		if (motor_running) {
			send_next_status_at = millis() + 5000LL;
		} else {
			send_next_status_at = millis() + 15 * 60 * 1000LL; // 15 minutes
		}
		send_cover_status();
		
		// Read and send INA226 measurements
		if (ina226_initialized) {
			bool was_sleeping = !motor_running;
			
			// Wake up INA226 if it was sleeping for measurement
			if (was_sleeping) {
				ina226_wake();
				delay(50); // Allow time for measurements to stabilize
			}
			
			// Read bus voltage (mV) and apply correction for counterfeit chip
			float bus_voltage_v = ina226.getBusVoltage() / INA226_VOLTAGE_CORRECTION;
			uint16_t voltage_mv = (uint16_t)(bus_voltage_v * 1000.0);
			send_battery_voltage(voltage_mv);
			
			// Read signed current (mA) - positive = discharging, negative = charging
			float current_a = ina226.getCurrent();
			int16_t current_ma = (int16_t)(current_a * 1000.0);
			send_battery_current(current_ma);
			
			// Read power (mW) and apply voltage correction
			float power_w = ina226.getPower() / INA226_VOLTAGE_CORRECTION;
			int32_t power_mw = (int32_t)(power_w * 1000.0f);

			// Apply sign of current since power appears to be unsigned in INA226
			power_mw = (current_a >= 0.0f) ? power_mw : -power_mw;

			send_battery_power(power_mw);
			
			Serial.print("INA226: ");
			Serial.print(bus_voltage_v, 3);
			Serial.print(F("V, "));
			Serial.print(current_a, 3);
			Serial.print(F("A"));
			Serial.print(F(", "));
			Serial.print(power_mw, 3);
			Serial.println(F("mW"));
			
			// Put INA226 back to sleep if motor is not running
			if (was_sleeping) {
				ina226_sleep();
			}
		}
		
		// Read and send BTN7960B motor current (only when motor is running)
		/*if (motor_running) {
			uint16_t motor_current_ma = read_motor_current_ma();
			send_motor_current(motor_current_ma);
			
			Serial.print("BTN7960B Motor Current: ");
			Serial.print(motor_current_ma);
			Serial.println(F("mA"));
		}*/
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
				radio_send(PIPE_TEMPERATURE, 'T', identification_letter, 0xFF, 0xFF);
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

			// Send temperature with retries
			bool fail = false;
			int retry_count = 3;
			while (retry_count--) {
				fail = radio_send(PIPE_TEMPERATURE, 'T', identification_letter, (raw >> 8) & 0xFF, raw & 0xFF);
				if (!fail) {
					break;
				}
				// Indicate failure to receive ACK
				radio_send(PIPE_TEMPERATURE, 'F', identification_letter, retry_count, 0);
				Sleepy::loseSomeTime(512L);
			}
		}
		
		if (thermometer_count == 0) {
			Serial.println("No DS18B20 thermometers found");
		}
	}

	// Check for millis() overflow prevention reboot
	if (millis() >= REBOOT_AT_MILLIS && !motor_running) {
		software_reboot();
	}

	// Power management: calculate optimal sleep time
	if (!motor_running && !radio_packet_received) {
		// Calculate time until next required action
		unsigned long current_time = millis();
		unsigned long sleep_time_temeprature = send_next_temperature_at - current_time;
		unsigned long sleep_time_status = send_next_status_at - current_time;

		unsigned long sleep_time = min(sleep_time_temeprature, sleep_time_status);
		
		if (sleep_time > 32768L) { // Max 32 seconds sleep
			sleep_time = 32768L;
		} else if (sleep_time < 128L) { // Min 128ms sleep
			sleep_time = 128L;
		}
		
		// Sleep for calculated time (power saving)
		Serial.print("Sleeping for ");
		Serial.print(sleep_time);
		Serial.println("ms");
		Serial.flush();
		Sleepy::loseSomeTime(sleep_time);
	}
}



