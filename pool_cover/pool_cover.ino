#include <SPI.h>
#include <JeeLib.h>
#include <OneWire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <INA226.h>
#include "printf.h" 
#include "KY040.h"

// Comment out this define to build without RF24 radio support.
// #define HAS_RF24
#define HAS_BUTTON_CMD
#define HAS_SERIAL_CONSOLE
#define DEBUG_ENCODER

#if defined(HAS_RF24) && defined(HAS_BUTTON_CMD)
#error "HAS_RF24 and HAS_BUTTON_CMD are mutually exclusive because they share pins."
#endif

#ifdef HAS_RF24
#include "nRF24L01.h"
#include "RF24.h"
#endif

// Motor is 17RPM not 27RPM 
/*
 * Pin Assignment for Arduino Pro Mini:
 * 
 * Digital Pins:
 *   0  - RX (Serial) - Reserved for programming/debugging
 *   1  - TX (Serial) - Reserved for programming/debugging  
 *   2  - Encoder CLK (interrupt capable)
 *   3  - RF24 IRQ (interrupt capable) - for radio packet reception
 *   4  - Encoder DT (direction)
 *   5  - Motor PWM A (PWM)
 *   6  - Motor PWM B (PWM)
 *   7  - DS18BI20 temperature sensor
 *   8  - RF24 CSN
 *   9  - RF24 CE (PWM) / UP button
 *   10 - Available
 *   11 - RF24 MOSI (PWM, SPI)
 *   12 - RF24 MISO (SPI) / DOWN button
 *   13 - RF24 SCK (SPI) + onboard LED / STOP button
 * 
 * Analog Pins:
 *   A0 - Motor Enable (both BTN7960B enables connected together)
 *   A1 - MCU battery voltage sense (via divider)
 *   A2 - MCU battery divider enable (LOW=enabled, Hi-Z=disabled)
 *   A3 - Available
 *   A4 - I2C SDA (INA226)
 *   A5 - I2C SCL (INA226)
 *   A6 - Available
 *   A7 - Available
 */

const int SCL_PIN = A5;        // I2C for INA226
const int SDA_PIN = A4;        // I2C for INA226
const int INA226_ALERT_PIN = 10; // INA226 alert pin (Pin Change Interrupt)

const int CE_PIN = 9;          // RF24 CE
const int CSN_PIN = 8;         // RF24 CSN
const int RF24_IRQ_PIN = 3;    // RF24 IRQ pin for external interrupts
const int RF24_MOSI_PIN = 11; // RF24 MOSI (SPI)
const int RF24_MISO_PIN = 12;  // RF24 MISO (SPI)
const int RF24_SCK_PIN = 13;   // RF24 SCK (SPI)
const int DS18B20_PIN = 7;     // DS18B20 temperature sensor

#ifdef HAS_BUTTON_CMD
const int BUTTON_UP_PIN = CE_PIN;
const int BUTTON_DOWN_PIN = RF24_MISO_PIN;
const int BUTTON_STOP_PIN = RF24_SCK_PIN;
#endif

// H-bridge motor control pins
const int MOTOR_PWM_A_PIN = 5;    // PWM pin for motor A (H-bridge left side)
const int MOTOR_PWM_B_PIN = 6;    // PWM pin for motor B (H-bridge right side) 
const int MOTOR_ENABLE_PIN = A0;  // Enable pin for both BTN7960B (connected together)
const int MCU_BATTERY_SENSE_PIN = A1;
const int MCU_BATTERY_DIVIDER_ENABLE_PIN = A2;

// KY-040 Encoder pins
const int ENCODER_CLK_PIN = 2;  // Interrupt pin (CLK)
const int ENCODER_DT_PIN = 4;   // Direction pin (DT)


OneWire ds(DS18B20_PIN);
const uint8_t INA226_I2C_ADDRESS = 0x40;
INA226 ina226(INA226_I2C_ADDRESS); 

// KY-040 Encoder object
KY040 encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN); 

// Overcurrent protection variables
static unsigned long overcurrent_start_time = 0;
static bool overcurrent_detected = false;
static const float OVERCURRENT_THRESHOLD = 6.0; // 6 Amperes
static const unsigned long OVERCURRENT_TIMEOUT = 15000; // 15 seconds

// INA226 alert handling
volatile bool ina226_alert_triggered = false;

#ifdef HAS_RF24
RF24 radio(CE_PIN, CSN_PIN, 4000000UL);
const uint64_t pipe_address_cover = 0xF0F0F0F0F1LL;
const uint64_t pipe_address_temperature = 0xF0F0F0F0F4LL;
#endif
#define PIPE_POOL_COVER 1
#define PIPE_TEMPERATURE 4

static unsigned long motor_duration_up = (10 * 60 + 30) * 1000UL; // 10 minutes 30 seconds for up
static unsigned long motor_duration_down = 8 * 60 * 1000UL; // max 8 minutes for down
static unsigned long motor_stop_at = 0;
static bool motor_running = false;
static char motor_direction = 'U';
static unsigned long send_next_temperature_at = 0; // Time for next temperature send
static unsigned long send_next_status_at = 0; // Time for next status send

// Reboot mechanism to avoid millis() overflow issues
const unsigned long REBOOT_AT_MILLIS = 4200000000UL; // ~48.6 days

// RF24 interrupt handling
volatile bool radio_packet_received = false;

// Radio configuration monitoring
static unsigned long check_radio_config_at = 0;
static const unsigned long RADIO_CONFIG_CHECK_INTERVAL_MOTOR = 5000;    // 5 seconds when motor running
static const unsigned long RADIO_CONFIG_CHECK_INTERVAL_IDLE = 3600000;  // 1 hour when idle
static uint8_t boot_mcusr_raw = 0;

#ifdef HAS_BUTTON_CMD
static const unsigned long BUTTON_LONG_PRESS_MS = 500UL;
static unsigned long button_up_pressed_at = 0;
static unsigned long button_down_pressed_at = 0;
static bool button_up_long_press_handled = false;
static bool button_down_long_press_handled = false;
static bool combo_top_latched = false;
static bool combo_bottom_latched = false;
static bool stop_button_was_pressed = false;
static bool up_button_was_pressed = false;
static bool down_button_was_pressed = false;
volatile bool button_interrupt_received = false;
#endif

// Encoder and position tracking variables
const long DEFAULT_TRAVEL_DISTANCE = 167; // Based on actual measurement, with 15 steps/turn = 10 turns
volatile long encoder_position = 0;  // Current encoder position
static long encoder_target_position = 0;  // Target position for current movement
static long encoder_top_position = DEFAULT_TRAVEL_DISTANCE;      // Marked top position
static long encoder_bottom_position = -DEFAULT_TRAVEL_DISTANCE; // Marked bottom position
static bool has_marked_top = false;
static bool has_marked_bottom = false;

// Serial debug mode: enabled when running off UART power (voltage < 5V)
static bool serial_debug_mode = false;
static long last_debug_encoder_position = 0;  // For detecting encoder changes in debug mode
static unsigned long next_debug_status_at = 0;  // Periodic debug status
#ifdef HAS_SERIAL_CONSOLE
static unsigned long last_serial_input_at = 0;
static const unsigned long SERIAL_CONSOLE_AWAKE_HOLDOFF_MS = 60000UL; // 1 minute
static const unsigned long SERIAL_CONSOLE_POLL_SLEEP_MS = 128UL;
#endif

// Debug: track raw encoder pin states
volatile byte last_encoder_state = 0xFF;
volatile unsigned long encoder_cw_count = 0;
volatile unsigned long encoder_ccw_count = 0;
volatile unsigned long encoder_active_count = 0;
volatile unsigned long encoder_idle_count = 0;
volatile unsigned long encoder_state_hist[4] = {0, 0, 0, 0};  // Count of each state seen

// Debug: track sequence failures - record last N state transitions
#define STATE_LOG_SIZE 32
volatile byte state_log[STATE_LOG_SIZE];
volatile byte state_log_idx = 0;

// Debug: track which interrupt source fires
volatile unsigned long int_clk_count = 0;  // External INT on pin 2 (CLK)
volatile unsigned long int_dt_count = 0;   // PCINT on pin 4 (DT)


int init_failed = 0;
bool ina226_initialized = false;

// Thermometer address to identification letter mapping
const struct {
    uint8_t addr[8];
    uint8_t letter;
} thermometer_letter_from_addr[] = {
    /*{{ 0x28, 0xff, 0xf0, 0x19, 0x91, 0x15, 0x01, 0x42 }, 'T'}, //test
	{{ 0x28, 0xFF, 0x9A, 0xEA, 0x90, 0x15, 0x01, 0x75 }, 'T'}, //test too*/
	// {{ 0x28, 0x84, 0xCC, 0xDC, 0x04, 0x00, 0x00, 0xA6 }, 'T'}, //test
	{{ 0x28, 0xD7, 0xC4, 0xD9, 0x04, 0x00, 0x00, 0x6E }, 'P'}, // swimming pool (water)
	{{ 0x28, 0x1D, 0x0B, 0x79, 0xA2, 0x00, 0x03, 0x1F }, 'e'}, // exterior (air)

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
const float SHUNT_RESISTANCE_OHMS = 0.010f;       // 10 milliohm shunt (R010)
const float INA226_SHUNT_VOLTAGE_LSB = 0.0000025f; // 2.5µV per LSB for shunt voltage register

// CLK interrupt handler (external INT on pin 2)
void encoder_clk_interrupt() {
    int_clk_count++;
    // Minimal handler - just call KY040 library directly
    switch (encoder.getRotation()) {
        case KY040::CLOCKWISE:
            encoder_position++;
            encoder_cw_count++;
            break;
        case KY040::COUNTERCLOCKWISE:
            encoder_position--;
            encoder_ccw_count++;
            break;
    }
}

// DT interrupt handler (PCINT on pin 4)
void encoder_dt_interrupt() {
    int_dt_count++;
    // Minimal handler - just call KY040 library directly
    switch (encoder.getRotation()) {
        case KY040::CLOCKWISE:
            encoder_position++;
            encoder_cw_count++;
            break;
        case KY040::COUNTERCLOCKWISE:
            encoder_position--;
            encoder_ccw_count++;
            break;
    }
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

#ifdef HAS_BUTTON_CMD
void button_interrupt() {
	button_interrupt_received = true;
}
#endif

// Safe read of encoder position (with interrupt protection)
long get_encoder_position() {
    long position;
    cli();
    position = encoder_position;
    sei();
    return position;
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

uint8_t encode_reset_cause(uint8_t mcusr) {
	if (mcusr & _BV(WDRF)) {
		return 'w'; // Watchdog reset
	}
	if (mcusr & _BV(BORF)) {
		return 'v'; // Brown-out (voltage dip) reset
	}
	if (mcusr & _BV(EXTRF)) {
		return 'e'; // External reset
	}
	if (mcusr & _BV(PORF)) {
		return 0;   // Normal power-on boot
	}
	return 'u';   // Unknown/undetermined
}

uint8_t get_reset_cause() {
	boot_mcusr_raw = MCUSR;
	MCUSR = 0;
	// Ensure watchdog is not left running after a watchdog reset.
	wdt_disable();
	return encode_reset_cause(boot_mcusr_raw);
}

void print_reset_cause(uint8_t reset_cause) {
	Serial.print(F("Reset cause flags:"));
	if (boot_mcusr_raw & _BV(WDRF)) Serial.print(F(" WDRF"));
	if (boot_mcusr_raw & _BV(BORF)) Serial.print(F(" BORF"));
	if (boot_mcusr_raw & _BV(EXTRF)) Serial.print(F(" EXTRF"));
	if (boot_mcusr_raw & _BV(PORF)) Serial.print(F(" PORF"));
	if (boot_mcusr_raw == 0) Serial.print(F(" none"));
	
	Serial.print(F(" | encoded=0x"));
	if (reset_cause < 0x10) Serial.print(F("0"));
	Serial.println(reset_cause, HEX);
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
	// Ramp-down to reduce voltage transients (~100ms total)
	for (int pwm = 255; pwm >= 0; pwm -= 2) {
		analogWrite(MOTOR_PWM_A_PIN, (motor_direction == 'U') ? pwm : 0);
		analogWrite(MOTOR_PWM_B_PIN, (motor_direction == 'D') ? pwm : 0);
		delayMicroseconds(780);
	}
	
	digitalWrite(MOTOR_ENABLE_PIN, LOW);
	analogWrite(MOTOR_PWM_A_PIN, 0);
	analogWrite(MOTOR_PWM_B_PIN, 0);
	motor_running = false;
	
	// Reset overcurrent protection
	overcurrent_detected = false;
	
	// Put INA226 to sleep when motor stops
	ina226_sleep();
	
	// Check if radio and INA226 survived motor shutdown
	delay(50);  // Let power settle
	check_radio_config_at = millis() + 5000;  // Force immediate config checks
	
	send_next_status_at = millis();
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
	long current_pos = get_encoder_position();
	
	// Determine actual upper and lower limits from marked positions
	long upper_limit, lower_limit;
	bool has_upper_limit = false;
	bool has_lower_limit = false;
	
	if (has_marked_top && has_marked_bottom) {
		// Both marked - use actual values to determine which is upper/lower
		if (encoder_top_position >= encoder_bottom_position) {
			upper_limit = encoder_top_position;
			lower_limit = encoder_bottom_position;
		} else {
			upper_limit = encoder_bottom_position;
			lower_limit = encoder_top_position;
		}
		has_upper_limit = true;
		has_lower_limit = true;
	} else if (has_marked_top) {
		// Only top marked - could be upper or lower depending on current position
		if (encoder_top_position >= current_pos) {
			upper_limit = encoder_top_position;
			has_upper_limit = true;
		} else {
			lower_limit = encoder_top_position;
			has_lower_limit = true;
		}
	} else if (has_marked_bottom) {
		// Only bottom marked - could be upper or lower depending on current position
		if (encoder_bottom_position <= current_pos) {
			lower_limit = encoder_bottom_position;
			has_lower_limit = true;
		} else {
			upper_limit = encoder_bottom_position;
			has_upper_limit = true;
		}
	}
	
	if (direction == 'U') {
		// Moving up - target is upper limit or current + default travel
		if (has_upper_limit) {
			encoder_target_position = upper_limit;
			if (current_pos >= encoder_target_position) {
				// Already at the "top" position? Add 1
				encoder_target_position = current_pos + 1;
			}
		} else {
			encoder_target_position = current_pos + DEFAULT_TRAVEL_DISTANCE;
		}
	} else { // direction == 'D'
		// Moving down - target is lower limit or current - default travel
		if (has_lower_limit) {
			encoder_target_position = lower_limit;
			if (current_pos <= encoder_target_position) {
				// Already at or past the lower limit - only allow 1 step
				encoder_target_position = current_pos - 1;
			}
		} else {
			encoder_target_position = current_pos - DEFAULT_TRAVEL_DISTANCE;
		}
	}
	
	send_cover_status();
	// Start 5-second cycle for frequent updates while motor running
	send_next_status_at = millis() + 5000LL;
}


int radio_send(uint8_t pipe_id, uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
#ifdef HAS_RF24
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
	if (serial_debug_mode) {
		Serial.print(F("radio_send p"));
		Serial.print(pipe_id);
		Serial.print(F("["));
		Serial.print((char)p0);
		Serial.print(F(" 0x"));
		if (p1 < 16)
			Serial.print(F("0"));
		Serial.print(p1, HEX);
		Serial.print(F(" 0x"));
		if (p2 < 16)
			Serial.print(F("0"));
		Serial.print(p2, HEX);
		Serial.print(F(" 0x"));
		if (p3 < 16)
			Serial.print(F("0"));
		Serial.print(p3, HEX);
		Serial.print(F("] -> "));
		Serial.println(ok ? F("OK") : F("KO"));
	}

	// Resume listening after sending
	radio.startListening();

	if (!ok) {
		// Radio send failed - trigger immediate config checks (both radio and INA226)
		check_radio_config_at = millis();
		return -1;
	}
	
	delay(5); // Hopefully this makes radio more reliable?
	return 0;
#else
	(void)pipe_id;
	(void)p0;
	(void)p1;
	(void)p2;
	(void)p3;
	return 0;
#endif
}

uint16_t read_mcu_battery_voltage_mv()
{
	// Enable resistor divider (pull pin to GND)
	pinMode(MCU_BATTERY_DIVIDER_ENABLE_PIN, OUTPUT);
	digitalWrite(MCU_BATTERY_DIVIDER_ENABLE_PIN, LOW);

	delay(2); // Let divider node settle before ADC conversion.
	uint16_t raw = analogRead(MCU_BATTERY_SENSE_PIN);

	// Disable resistor divider (pin in HIGH-Z)
	pinMode(MCU_BATTERY_DIVIDER_ENABLE_PIN, INPUT);
	// Arduino ADC: 0-1023 maps to 0-Vref. Controller is powered from 3.3V LDO.
	uint32_t millivolts = (uint32_t)raw * 3300UL / 1023UL;
	// Vsource = Vtap * (Rtop + Rbottom) / Rbottom, with Rtop=2.2k and Rbottom=6.8k.
	millivolts = (millivolts * (2200UL + 6800UL)) / 6800UL;
	return (uint16_t)millivolts;
}

void send_main_battery_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_POOL_COVER, 'V', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
}

void send_mcu_battery_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_POOL_COVER, 'v', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
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
		send_encoder_position((uint32_t)get_encoder_position());
	}
}

static void mark_top_position() {
	encoder_top_position = get_encoder_position();
	has_marked_top = true;
	Serial.print(F("Top marked at encoder position: "));
	Serial.println(encoder_top_position);
}

static void mark_bottom_position() {
	encoder_bottom_position = get_encoder_position();
	has_marked_bottom = true;
	Serial.print(F("Bottom marked at encoder position: "));
	Serial.println(encoder_bottom_position);
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
					mark_top_position();
				} else if (param2 == 'D') {
					mark_bottom_position();
				}
				break;
			case 'Q': // Query status
				send_cover_status();
				break;
			default:
				Serial.print(F("Unknown cover command: "));
				Serial.println((char)cmd);
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

#ifdef HAS_BUTTON_CMD
static inline bool button_pressed(int pin) {
	// Buttons are wired to GND and use INPUT_PULLUP.
	return digitalRead(pin) == LOW;
}

void process_button_commands() {
	unsigned long now = millis();
	bool up_pressed = button_pressed(BUTTON_UP_PIN);
	bool down_pressed = button_pressed(BUTTON_DOWN_PIN);
	bool stop_pressed = button_pressed(BUTTON_STOP_PIN);

	if (up_pressed && !up_button_was_pressed) {
		Serial.println(F("Button UP pressed"));
	}
	if (down_pressed && !down_button_was_pressed) {
		Serial.println(F("Button DOWN pressed"));
	}
	if (stop_pressed && !stop_button_was_pressed) {
		Serial.println(F("Button STOP pressed -> stop motor"));
	}

	if (stop_pressed && !stop_button_was_pressed) {
		stop_motor();
	}
	up_button_was_pressed = up_pressed;
	down_button_was_pressed = down_pressed;
	stop_button_was_pressed = stop_pressed;

	// Combo actions are edge-triggered to avoid repeated re-marking while held.
	bool up_stop_combo = up_pressed && stop_pressed;
	if (up_stop_combo && !combo_top_latched) {
		Serial.println(F("Button combo UP+STOP -> mark top"));
		mark_top_position();
		combo_top_latched = true;
	}
	if (!up_stop_combo) {
		combo_top_latched = false;
	}

	bool down_stop_combo = down_pressed && stop_pressed;
	if (down_stop_combo && !combo_bottom_latched) {
		Serial.println(F("Button combo DOWN+STOP -> mark bottom"));
		mark_bottom_position();
		combo_bottom_latched = true;
	}
	if (!down_stop_combo) {
		combo_bottom_latched = false;
	}

	// Ignore long-press direction actions while STOP is held.
	if (up_pressed && !stop_pressed) {
		if (button_up_pressed_at == 0) {
			button_up_pressed_at = now;
			button_up_long_press_handled = false;
		} else if (!button_up_long_press_handled && (unsigned long)(now - button_up_pressed_at) >= BUTTON_LONG_PRESS_MS) {
			Serial.println(F("Button UP long press -> move UP"));
			start_motor('U');
			button_up_long_press_handled = true;
		}
	} else {
		button_up_pressed_at = 0;
		button_up_long_press_handled = false;
	}

	if (down_pressed && !stop_pressed) {
		if (button_down_pressed_at == 0) {
			button_down_pressed_at = now;
			button_down_long_press_handled = false;
		} else if (!button_down_long_press_handled && (unsigned long)(now - button_down_pressed_at) >= BUTTON_LONG_PRESS_MS) {
			Serial.println(F("Button DOWN long press -> move DOWN"));
			start_motor('D');
			button_down_long_press_handled = true;
		}
	} else {
		button_down_pressed_at = 0;
		button_down_long_press_handled = false;
	}
}
#endif

void check_radio_messages()
{
#ifdef HAS_RF24
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

// Radio configuration functions
#ifdef HAS_RF24
void configure_radio() {
	// Configure all radio settings
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
}

bool check_radio_configuration() {
	uint8_t radio_details[43];
	radio.encodeRadioDetails(radio_details);
	const uint8_t expected_config = 0x3F;      // PRIM_RX|PWR_UP|CRCO|EN_CRC + IRQ mask (RX enabled, TX/MAX_RT masked)
	const uint8_t expected_en_aa = 0x3F;       // Auto-ack enabled on all pipes
	const uint8_t expected_en_rxaddr = 0x12;   // Enable RX pipes 1 and 4
	const uint8_t expected_setup_aw = 0x03;    // 5-byte addresses
	const uint8_t expected_setup_retr = 0xFF;  // ARD=15, ARC=15
	const uint8_t expected_rf_ch = 80;         // 2.480 GHz
	const uint8_t expected_payload_width = 4;  // 4-byte payloads
	/*
	* | index | register/data |
	* |------:|:--------------|
	* | 0 |     NRF_CONFIG |
	* | 1 |     EN_AA |
	* | 2 |     EN_RXADDR |
	* | 3 |     SETUP_AW |
	* | 4 |     SETUP_RETR |
	* | 5 |     RF_CH |
	* | 6 |     RF_SETUP |
	* | 7 |     NRF_STATUS |
	* | 8 |     OBSERVE_TX |
	* | 9 |     CD (aka RPD) |
	* | 10-14 | RX_ADDR_P0 |
	* | 15-19 | RX_ADDR_P1 |
	* | 20 |    RX_ADDR_P2 |
	* | 21 |    RX_ADDR_P3 |
	* | 22 |    RX_ADDR_P4 |
	* | 23 |    RX_ADDR_P5 |
	* | 24-28 | TX_ADDR |
	* | 29 |    RX_PW_P0 |
	* | 30 |    RX_PW_P1 |
	* | 31 |    RX_PW_P2 |
	* | 32 |    RX_PW_P3 |
	* | 33 |    RX_PW_P4 |
	* | 34 |    RX_PW_P5 |
	* | 35 |    FIFO_STATUS |
	* | 36 |    DYNPD |
	* | 37 |    FEATURE |
	* | 38-39 | ce_pin |
	* | 40-41 | csn_pin |
	* | 42 |    SPI speed (in MHz) or'd with (isPlusVariant << 4) |
	*/

	// Check stable register values that should not drift in normal operation.
	// Intentionally do NOT check dynamic fields such as STATUS, OBSERVE_TX, FIFO_STATUS,
	// TX_ADDR, and RX_ADDR_P0 because those can change at runtime without indicating corruption.
	if (radio_details[0] != expected_config) {
		Serial.println(F("Radio config mismatch on CONFIG"));
		goto mismatch;
	}

	if (radio_details[1] != expected_en_aa) {
		Serial.println(F("Radio config mismatch on EN_AA"));
		goto mismatch;
	}

	if (radio_details[2] != expected_en_rxaddr) {
		Serial.println(F("Radio config mismatch on EN_RXADDR"));
		goto mismatch;
	}

	if (radio_details[3] != expected_setup_aw) {
		Serial.println(F("Radio config mismatch on SETUP_AW"));
		goto mismatch;
	}

	if (radio_details[4] != expected_setup_retr) {
		Serial.println(F("Radio config mismatch on SETUP_RETR"));
		goto mismatch;
	}

	// Check RF_CH register (array index 5)
	if (radio_details[5] != expected_rf_ch) {
		Serial.println(F("Radio config mismatch on RF_CH"));
		goto mismatch;
	}
	
	if (radio.getDataRate() != RF24_250KBPS) {
		Serial.println(F("Radio config mismatch on data rate"));
		goto mismatch;
	}
	
	if (radio.getCRCLength() != RF24_CRC_16) {
		Serial.println(F("Radio config mismatch on CRC length"));
		goto mismatch;
	}
	
	if (radio.getPALevel() != RF24_PA_MAX) {
		Serial.println(F("Radio config mismatch on PA level"));
		goto mismatch;
	}

	// Static payload mode should stay fixed at 4 bytes on active RX pipes.
	if (radio_details[30] != expected_payload_width) {
		Serial.println(F("Radio config mismatch on RX_PW_P1"));
		goto mismatch;
	}

	if (radio_details[33] != expected_payload_width) {
		Serial.println(F("Radio config mismatch on RX_PW_P4"));
		goto mismatch;
	}

	// Dynamic payload features are intentionally disabled.
	if (radio_details[36] != 0x00 || radio_details[37] != 0x00) {
		Serial.println(F("Radio config mismatch on DYNPD/FEATURE"));
		goto mismatch;
	}
	
	// Check RX_ADDR_P1 (array indices 15-19) - should match pipe_address_cover (0xF0F0F0F0F1LL)
	uint64_t expected_cover_addr = pipe_address_cover;
	for (int i = 0; i < 5; i++) {
		uint8_t expected_byte = (expected_cover_addr >> (i * 8)) & 0xFF;
		if (radio_details[15 + i] != expected_byte) {
			Serial.println(F("Radio config mismatch on RX_ADDR_P1"));
			goto mismatch;
		}
	}
	
	// Check RX_ADDR_P4 (array index 22) - should match last byte of pipe_address_temperature (0xF4)
	uint8_t expected_temp_lsb = pipe_address_temperature & 0xFF;
	if (radio_details[22] != expected_temp_lsb) {
		Serial.println(F("Radio config mismatch on RX_ADDR_P4"));
		goto mismatch;
	}
	
	return true;

mismatch:
	radio.printDetails();
	return false;
}

void reset_radio_configuration() {
	Serial.println(F("Resetting radio"));
	
	// Stop listening first
	radio.stopListening();
	
	// Power cycle the radio
	radio.powerDown();
	delay(10);
	radio.powerUp();
	delay(10);
	
	// Apply all radio configuration settings
	configure_radio();
	
	// Send booting message
	radio_send(PIPE_POOL_COVER, 'Q', 'b', 'r', 0);
}
#endif


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

// Helper function to configure INA226 with calibration and alerts
bool configure_ina226() {
	// Calculate max current based on INA226 library constraint
	// Library checks: maxCurrent * shunt <= 0.08190V (81.90mV)
	float max_current = 0.08190 / SHUNT_RESISTANCE_OHMS;  // Maximum current the library will accept
	
	Serial.print(F("Calibrating with max current: "));
	Serial.print(max_current);
	Serial.print(F("A with "));
	Serial.print(SHUNT_RESISTANCE_OHMS, 4);
	Serial.print(F(" ohm shunt... "));
	
	int cal_result = ina226.setMaxCurrentShunt(max_current, SHUNT_RESISTANCE_OHMS, true);
	if (cal_result != 0) {
		Serial.print(F("FAILED (0x"));
		Serial.print(cal_result, HEX);
		Serial.println(F(")"));
		return false;
	}
	
	Serial.println(F("OK"));
	
	Serial.print(F("INA226 calibrated. Current LSB: "));
	Serial.print(ina226.getCurrentLSB_mA());
	Serial.println(F(" mA"));
	
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
		return false;
	}
	
	// Configure alert register for shunt overvoltage (overcurrent)
	if (!ina226.setAlertRegister(INA226_SHUNT_OVER_VOLTAGE)) {
		Serial.println(F("ERROR: Failed to configure INA226 alert register"));
		return false;
	}
	
	Serial.print(F("INA226 alert configured for overcurrent at "));
	Serial.print(OVERCURRENT_THRESHOLD);
	Serial.println(F("A"));
	
	return true;
}

void read_thermometers(bool send_radio) {
	uint8_t addr[8];
	uint8_t thermometer_count = 0;

	if (serial_debug_mode) {
		Serial.println(F("Temperature scan..."));
	}

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

		ds.reset();
		ds.select(addr);
		ds.write(0xBE); // Read scratchpad

		uint8_t data[9];
		for (int i = 0; i < 9; i++) {
			data[i] = ds.read();
		}

		// Check CRC
		if (data[8] != OneWire::crc8(data, 8)) {
			if (serial_debug_mode) {
				Serial.print(F("Thermometer "));
				Serial.print((char)identification_letter);
				Serial.println(F(": CRC error"));
			} else {
				Serial.print(F("ERROR: CRC mismatch for thermometer "));
				Serial.println((char)identification_letter);
			}

			// Indicate that we read garbage
			if (send_radio) {
				radio_send(PIPE_TEMPERATURE, 'T', identification_letter, 0xFF, 0xFF);
			}
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
		if (serial_debug_mode) {
			Serial.print(F("Thermometer "));
			Serial.print((char)identification_letter);
			Serial.print(F(": "));
			Serial.print(temperature_c, 2);
			Serial.println(F(" C"));
		} else {
			printf("Temperature %c is %d\n", identification_letter, (int)(100.0 * temperature_c));
		}

		if (send_radio) {
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
	}

	if (thermometer_count == 0) {
		Serial.println(F("No DS18B20 thermometers found"));
	}
}

void read_battery_information(bool send_radio, bool print_output) {
	uint16_t mcu_mv = read_mcu_battery_voltage_mv();
	if (send_radio) {
		send_mcu_battery_voltage(mcu_mv);
	}
	if (print_output) {
		Serial.print(F("MCU battery: "));
		Serial.print(mcu_mv);
		Serial.println(F(" mV"));
	}

	if (!ina226_initialized) {
		if (print_output) {
			Serial.println(F("Main battery: INA226 not initialized"));
		}
		return;
	}

	bool was_sleeping = !motor_running;
	if (was_sleeping) {
		ina226_wake();
		delay(50);
	}

	// Read bus voltage (mV) and apply correction for counterfeit chip
	float bus_voltage_v = ina226.getBusVoltage() / INA226_VOLTAGE_CORRECTION;
	uint16_t voltage_mv = (uint16_t)(bus_voltage_v * 1000.0);

	// Enable serial debug mode if running off UART power (voltage < 5V)
	serial_debug_mode = (bus_voltage_v < 5.0);
#ifdef HAS_SERIAL_CONSOLE
	serial_debug_mode = true;
#endif

	// Read signed current (mA) - positive = discharging, negative = charging
	float current_a = ina226.getCurrent();
	int16_t current_ma = (int16_t)(current_a * 1000.0);

	// Read power (mW) and apply voltage correction
	float power_w = ina226.getPower() / INA226_VOLTAGE_CORRECTION;
	int32_t power_mw = (int32_t)(power_w * 1000.0f);
	// Apply sign of current since power appears to be unsigned in INA226
	power_mw = (current_a >= 0.0f) ? power_mw : -power_mw;

	if (send_radio) {
		send_main_battery_voltage(voltage_mv);
		send_battery_current(current_ma);
		send_battery_power(power_mw);
	}

	if (print_output) {
		Serial.print("INA226: ");
		Serial.print(bus_voltage_v, 3);
		Serial.print(F("V, "));
		Serial.print(current_a, 3);
		Serial.print(F("A"));
		Serial.print(F(", "));
		Serial.print(power_mw);
		Serial.println(F("mW"));
	}

	// Put INA226 back to sleep if motor is not running
	if (was_sleeping) {
		ina226_sleep();
	}
}

#ifdef HAS_SERIAL_CONSOLE
bool serial_console_holdoff_active() {
	return (unsigned long)(millis() - last_serial_input_at) < SERIAL_CONSOLE_AWAKE_HOLDOFF_MS;
}

void print_serial_console_help() {
	Serial.println(F("Serial console commands:"));
	Serial.println(F("  H: Help"));
	Serial.println(F("  U: Up"));
	Serial.println(F("  D: Down"));
	Serial.println(F("  C: Close (alias for Down)"));
	Serial.println(F("  S: Stop"));
	Serial.println(F("  R: Reset radio"));
	Serial.println(F("  P: Print/check radio config"));
	Serial.println(F("  B: Battery information"));
	Serial.println(F("  T: Temperature information"));
	Serial.println(F("  M: Mark top"));
	Serial.println(F("  m: Mark bottom"));
}

void handle_serial_console_command(char cmd) {
	switch (cmd) {
		case 'H':
			print_serial_console_help();
			break;
		case 'U':
			Serial.println(F("Console: Up"));
			start_motor('U');
			break;
		case 'D':
			Serial.println(F("Console: Down"));
			start_motor('D');
			break;
		case 'C':
			Serial.println(F("Console: Close"));
			start_motor('D');
			break;
		case 'S':
			Serial.println(F("Console: Stop"));
			stop_motor();
			break;
		case 'R':
			Serial.println(F("Console: Reset radio"));
#ifdef HAS_RF24
			reset_radio_configuration();
#else
			Serial.println(F("RF24 disabled"));
#endif
			break;
		case 'P':
			Serial.println(F("Console: Print/check radio config"));
#ifdef HAS_RF24
			radio.printDetails();
			Serial.println(check_radio_configuration() ? F("Radio config: OK") : F("Radio config: MISMATCH"));
#else
			Serial.println(F("RF24 disabled"));
#endif
			break;
		case 'B':
			Serial.println(F("Console: Battery information"));
			read_battery_information(false, true);
			break;
		case 'T':
			Serial.println(F("Console: Temperature information"));
			read_thermometers(false);
			break;
		case 'M':
			mark_top_position();
			break;
		case 'm':
			mark_bottom_position();
			break;
		default:
			Serial.print(F("Unknown command: "));
			Serial.println(cmd);
			print_serial_console_help();
			break;
	}
}

bool process_serial_console() {
	bool processed_input = false;
	while (Serial.available() > 0) {
		char cmd = (char)Serial.read();
		last_serial_input_at = millis();
		processed_input = true;
		if (cmd == '\n' || cmd == '\r') {
			continue;
		}
		handle_serial_console_command(cmd);
	}
	return processed_input;
}
#endif

void setup(){
	uint8_t boot_reset_cause = get_reset_cause();
	printf_begin();
	Serial.begin(115200);
	Serial.println(F("Pool cover controller starting..."));  
	print_reset_cause(boot_reset_cause);

	// H-bridge motor control pins
	pinMode(MOTOR_PWM_A_PIN, OUTPUT);
	pinMode(MOTOR_PWM_B_PIN, OUTPUT);
	pinMode(MOTOR_ENABLE_PIN, OUTPUT);

	// MCU battery sensing
	pinMode(MCU_BATTERY_SENSE_PIN, INPUT);
	pinMode(MCU_BATTERY_DIVIDER_ENABLE_PIN, INPUT);

	// INA226 alert pin
	pinMode(INA226_ALERT_PIN, INPUT_PULLUP);
	// Attach Pin Change Interrupt to INA226 alert pin (pin 10)
	attachPCINT(digitalPinToPCINT(INA226_ALERT_PIN), ina226_interrupt, FALLING);

	// Encoder pins
	pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
	pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
	
	// Set up interrupts for encoder using KY040 library
	// Pin 2 supports external interrupt (INT0), pin 4 only supports PCINT
	attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoder_clk_interrupt, CHANGE);  // External INT on pin 2 (CLK)
	attachPCINT(digitalPinToPCINT(ENCODER_DT_PIN), encoder_dt_interrupt, CHANGE);           // PCINT on pin 4 (DT)

#ifdef HAS_RF24
	// RF24 IRQ pin
	pinMode(RF24_IRQ_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(RF24_IRQ_PIN), rf24_interrupt, FALLING);

	// Radio init
	radio.begin();
	radio.powerDown();
	
	// Apply all radio configuration settings
	configure_radio();

	
	if ((radio.getDataRate() != RF24_250KBPS) ||
		(radio.getCRCLength() != RF24_CRC_16) /*|| 
		(radio.getChannel() != 95)*/) {
		// failed to initialize radio
		init_failed = 1;
	}
#elif defined(HAS_BUTTON_CMD)
	pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
	pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
	pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
	attachPCINT(digitalPinToPCINT(BUTTON_UP_PIN), button_interrupt, CHANGE);
	attachPCINT(digitalPinToPCINT(BUTTON_DOWN_PIN), button_interrupt, CHANGE);
	attachPCINT(digitalPinToPCINT(BUTTON_STOP_PIN), button_interrupt, CHANGE);
#endif

	// Initialize I2C for INA226
	Wire.begin();
	
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
	}
	
	// Configure INA226 if it was successfully initialized
	if (ina226_initialized) {
		
		if (!configure_ina226()) {
			Serial.println(F("ERROR: INA226 configuration failed"));
			ina226_initialized = false;
			init_failed = 1;
		}
	}

	Serial.println(F("Pool cover controller ready"));

	radio_send(PIPE_POOL_COVER, 'Q', 'b', boot_reset_cause, 0); //"booting"
	stop_motor();
	
	// Print initial encoder pin states for debug
	Serial.print(F("Encoder pins: CLK(D"));
	Serial.print(ENCODER_CLK_PIN);
	Serial.print(F(")="));
	Serial.print(digitalRead(ENCODER_CLK_PIN));
	Serial.print(F(" DT(D"));
	Serial.print(ENCODER_DT_PIN);
	Serial.print(F(")="));
	Serial.println(digitalRead(ENCODER_DT_PIN));
}

void loop() 
{
#ifdef DEBUG_ENCODER
	static unsigned long next_encoder_debug_dump_at = 0;
	if ((long)(millis() - next_encoder_debug_dump_at) >= 0) {
		next_encoder_debug_dump_at = millis() + 500UL;
		Serial.print(F("Encoder position: "));
		Serial.print(get_encoder_position());
		Serial.print(F(" int_clk_count: "));
		Serial.print(int_clk_count);
		Serial.print(F(" int_dt_count: "));
		Serial.println(int_dt_count);
	}
#endif

	if (radio_packet_received) {
		radio_packet_received = false; // Clear flag
		check_radio_messages();
	}

#ifdef HAS_SERIAL_CONSOLE
	if (process_serial_console()) {
		// Prioritize interactive console responsiveness over periodic tasks.
		return;
	}
#endif

#ifdef HAS_BUTTON_CMD
	process_button_commands();
#endif

	// Drive motor
	if (motor_running) {
		bool must_stop = false;
		
		// Check for INA226 alert (overcurrent)
		if (0 && ina226_initialized && ina226_alert_triggered) {
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
		if (0 && overcurrent_detected) {
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
		
		long current_encoder_pos = get_encoder_position();
		if (motor_direction == 'U') {
			// Moving up
			if (current_encoder_pos >= encoder_target_position) {
				Serial.print(F("Motor stopped: up position limit reached at "));
				Serial.println(current_encoder_pos);
				must_stop = true;
			}
		} else { // motor_direction == 'D'
			// Moving down
			if (current_encoder_pos <= encoder_target_position) {
				Serial.print(F("Motor stopped: down position limit reached at "));
				Serial.println(current_encoder_pos);
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
			send_next_status_at = millis() + 60 * 60 * 1000LL; // 1 hour
		}
		send_cover_status();
		read_battery_information(true, true);
	}

	// Send temperature
	if (millis() >= send_next_temperature_at) {
		send_next_temperature_at = millis() + 15 * 60 * 1000LL; // 15 minutes
		read_thermometers(true);
	}
	
	// Monitor and reset radio/INA226 configuration if corrupted by EMI
#ifdef HAS_RF24
	if (millis() >= check_radio_config_at) {
		unsigned long interval = motor_running ? RADIO_CONFIG_CHECK_INTERVAL_MOTOR : RADIO_CONFIG_CHECK_INTERVAL_IDLE;
		check_radio_config_at = millis() + interval;
		
		if (!check_radio_configuration()) {
			reset_radio_configuration();
		}
	}
#endif

	// Check for millis() overflow prevention reboot
	if (millis() >= REBOOT_AT_MILLIS && !motor_running) {
		software_reboot();
	}

	if (radio_packet_received) {
		return;
	}

#ifdef HAS_SERIAL_CONSOLE
	// If bytes arrived after process_serial_console() ran, service them on the next
	// loop iteration instead of entering a long sleep.
	if (Serial.available() > 0) {
		last_serial_input_at = millis();
		return;
	}
#endif

	// Power management: sleep
	// Calculate time until next required action
	unsigned long now = millis();
	unsigned long sleep_until;
	
	if (send_next_temperature_at > send_next_status_at) {
		sleep_until = send_next_status_at;
	} else {
		sleep_until = send_next_temperature_at;
	}

	if (motor_running) {
		Serial.flush();
		// Sleep a bit when the motor is running, but keep the radio listening all the time for a STOP packet
		Sleepy::loseSomeTime(128);
		return;
	}

	if (sleep_until <= now) {
		// No sleep needed, we are already past the next required action
		return;
	}

	unsigned long sleep_duration = sleep_until - now;

#ifdef HAS_SERIAL_CONSOLE
	// After any recent serial input, keep sleep slices short so "line hammering"
	// can hold the MCU responsive long enough to type full commands.
	// During holdoff, do not enter Sleepy at all.
	if (serial_console_holdoff_active()) {
		delay(10);
		return;
	}
#endif

#ifndef HAS_RF24
#ifdef HAS_BUTTON_CMD
	// In button-command mode, never sleep while any button is pressed.
	if (button_pressed(BUTTON_UP_PIN) || button_pressed(BUTTON_DOWN_PIN) || button_pressed(BUTTON_STOP_PIN)) {
		return;
	}
	button_interrupt_received = false;
	Serial.flush();
	Sleepy::loseSomeTime(sleep_duration);
	if (button_interrupt_received) {
		return;
	}
	return;
#else
	// No radio and no button commands: sleep directly until the next scheduled task.
	Serial.flush();
	Sleepy::loseSomeTime(sleep_duration);
	return;
#endif
#else
	const unsigned long RX_SLEEP_MS = 192;
	const unsigned long RX_LISTEN_MS = 48; // 16ms aligned, ~25% duty at 192ms cycle
	const unsigned long RX_CYCLE_MS = RX_SLEEP_MS + RX_LISTEN_MS;
	const unsigned long MEASURED_CYCLE_TIME = RX_CYCLE_MS + 6UL;

	Serial.print("Sleeping for ");
	Serial.print(sleep_duration);
	Serial.println("ms");
	Serial.flush();

	while (sleep_duration > 0) {
		// Sleep with a duty-cycled RX on RF24: 160ms radio off, 32ms radio RX on
		// measured consumption is about 23mA with RX on

		// Keep the radio listening for 32ms
		Sleepy::loseSomeTime(RX_LISTEN_MS);
		if (radio_packet_received) {
			return;
		}

		// Radio OFF phase
		radio.stopListening();
		radio.powerDown();
		Serial.flush();
		Sleepy::loseSomeTime(RX_SLEEP_MS);
		radio.powerUp();
		radio.startListening();

		if (sleep_duration <= MEASURED_CYCLE_TIME) {
			sleep_duration = 0;
		} else {
			sleep_duration -= MEASURED_CYCLE_TIME;
		}
	}
#endif
}



