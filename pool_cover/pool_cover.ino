#include <SPI.h>
#include <JeeLib.h>
#include <OneWire.h>
#include <avr/sleep.h>
#include <PinChangeInterrupt.h>  // Add PinChangeInterrupt library for INA226
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
 *   A0 - Motor Enable A (used as digital output)
 *   A1 - Motor Enable B (used as digital output)
 *   A2 - Available
 *   A3 - Available
 *   A4 - I2C SDA (INA226)
 *   A5 - I2C SCL (INA226)
 *   A6 - Motor Current Sense A (analog only)
 *   A7 - Motor Current Sense B (analog only)
 */

const int SCL_PIN = A5;        // I2C for INA226
const int SDA_PIN = A4;        // I2C for INA226
const int INA226_ALERT_PIN = 10; // INA226 alert pin (Pin Change Interrupt)

const int CE_PIN = 9;          // RF24 CE
const int CSN_PIN = 8;         // RF24 CSN
const int RF24_IRQ_PIN = 3;    // RF24 IRQ pin for external interrupts
const int DS18B20_PIN = 7;     // DS18B20 temperature sensor

// H-bridge motor control pins
const int MOTOR_PWM_A_PIN = 5;    // PWM pin for motor A (H-bridge)
const int MOTOR_PWM_B_PIN = 6;    // PWM pin for motor B (H-bridge) 
const int MOTOR_ENABLE_A_PIN = A0; // Enable pin for motor A (used as digital)
const int MOTOR_ENABLE_B_PIN = A1; // Enable pin for motor B (used as digital)
const int MOTOR_CURRENT_A_PIN = A6; // Current sense for motor A (analog only)
const int MOTOR_CURRENT_B_PIN = A7; // Current sense for motor B (analog only)

// KY-040 Encoder pins
const int ENCODER_CLK_PIN = 2;  // Interrupt pin (CLK)
const int ENCODER_DT_PIN = 4;   // Direction pin (DT)


OneWire ds(DS18B20_PIN);
INA226 ina226(0x44); 

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

static unsigned long motor_duration = 10 * 60 * 1000UL; // 10 minutes
static unsigned long motor_stop_at = 0;
static bool motor_running = false;
static char motor_direction = 'U';
static unsigned long send_next_temperature_at = 0; // Time for next temperature send
static unsigned long send_next_status_at = 0; // Time for next status send

// RF24 interrupt handling
volatile bool radio_packet_received = false;

// Encoder and position tracking variables
const long DEFAULT_TRAVEL_DISTANCE = 30 * 2 * 170; // 30 steps/turn * 2 turns/motor turn * 170 motor turns
volatile long encoder_position = 0;  // Current encoder position
static long encoder_target_position = 0;  // Target position for current movement
static long encoder_top_position = DEFAULT_TRAVEL_DISTANCE;     // Marked top position
static long encoder_bottom_position = -DEFAULT_TRAVEL_DISTANCE; // Marked bottom position (170 motor turns * 2)
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

// Encoder interrupt handler
void encoder_interrupt() {
	bool encoder_direction_cw;
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    
    // Debounce: ignore interrupts within 5ms
    if (interrupt_time - last_interrupt_time < 5) {
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
//    ina226_alert_triggered = true;
	// Disable until I have the correct shunt
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

void stop_motor()
{
	digitalWrite(MOTOR_ENABLE_A_PIN, LOW);
	digitalWrite(MOTOR_ENABLE_B_PIN, LOW);
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
	digitalWrite(MOTOR_ENABLE_A_PIN, LOW);
	digitalWrite(MOTOR_ENABLE_B_PIN, LOW);
	analogWrite(MOTOR_PWM_A_PIN, 0);
	analogWrite(MOTOR_PWM_B_PIN, 0);
	delay(100);
	
	// Set direction and start motor at full speed
	if (direction == 'U') {
		// Up direction: A enabled, B disabled
		analogWrite(MOTOR_PWM_A_PIN, 255);
		analogWrite(MOTOR_PWM_B_PIN, 0);
		digitalWrite(MOTOR_ENABLE_A_PIN, HIGH);
		digitalWrite(MOTOR_ENABLE_B_PIN, LOW);
	} else { // direction == 'D'
		// Down direction: A disabled, B enabled
		analogWrite(MOTOR_PWM_A_PIN, 0);
		analogWrite(MOTOR_PWM_B_PIN, 255);
		digitalWrite(MOTOR_ENABLE_A_PIN, LOW);
		digitalWrite(MOTOR_ENABLE_B_PIN, HIGH);
	}
	
	motor_running = true;
	motor_direction = direction;
	motor_stop_at = millis() + motor_duration;
	
	// Wake up INA226 for current monitoring
	ina226_wake();
	
	// Set target encoder position based on direction and limits
	long current_pos = encoder_position;
	
	if (direction == 'U') {
		// Moving up - target is top position or current + default travel
		if (has_marked_top) {
			encoder_target_position = encoder_top_position;
		} else {
			encoder_target_position = current_pos + DEFAULT_TRAVEL_DISTANCE;
		}
	} else { // direction == 'D'
		// Moving down - target is bottom position or current + default travel
		if (has_marked_bottom) {
			encoder_target_position = encoder_bottom_position;
		} else {
			encoder_target_position = current_pos - DEFAULT_TRAVEL_DISTANCE;
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



// Pool cover RF24 protocol functions
void send_battery_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_POOL_COVER, 'V', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
}

void send_solar_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_POOL_COVER, 'S', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
}

void send_battery_power(uint16_t power_mw)
{
	radio_send(PIPE_POOL_COVER, 'P', power_mw & 0xFF, (power_mw >> 8) & 0xFF, 0);
}

void send_battery_current(int16_t current_ma)
{
	radio_send(PIPE_POOL_COVER, 'I', current_ma & 0xFF, (current_ma >> 8) & 0xFF, 0);
}

void send_encoder_position(int32_t position)
{
	radio_send(PIPE_POOL_COVER, 'E', position & 0xFF, (position >> 8) & 0xFF, (position >> 16) & 0xFF);
}

void send_motor_temperature(int16_t temp_hundredths_c)
{
	radio_send(PIPE_POOL_COVER, 'T', temp_hundredths_c & 0xFF, (temp_hundredths_c >> 8) & 0xFF, 0);
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
					Serial.println(F("Mark travel top"));
					// Mark current position as top
					encoder_top_position = encoder_position;
					has_marked_top = true;
					Serial.print(F("Top marked at encoder position: "));
					Serial.println(encoder_top_position);
				} else if (param2 == 'D') {
					Serial.println(F("Mark travel bottom"));
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
		motor_duration = duration;
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
		
/*		Serial.print("Received on pipe ");
		Serial.print(pipe_num);
		Serial.print(": ");
		Serial.print((char)payload[0]);
		for (int i = 1; i < 4; i++) {
			Serial.print(payload[i], HEX);
			Serial.print(" ");
		}
		Serial.println();*/
		
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

// INA226 power management functions
void ina226_sleep() {
	if (ina226_initialized && ina226.isCalibrated()) {
		ina226.shutDown();
		Serial.println(F("INA226 entering sleep mode"));
	}
}

void ina226_wake() {
	if (ina226_initialized && ina226.isCalibrated()) {
		ina226.setModeShuntBusContinuous();
		// Re-configure alert after wake up
		uint16_t alert_limit = 28000;  // 0.7A threshold with 0.1Ω shunt
		ina226.setAlertLimit(alert_limit);
		ina226.setAlertRegister(INA226_SHUNT_OVER_VOLTAGE);
		Serial.println(F("INA226 waking up from sleep mode"));
	}
}

void setup(){
	printf_begin();
	Serial.begin(115200);
	Serial.println(F("Pool cover controller starting..."));  

	// H-bridge motor control pins
	pinMode(MOTOR_PWM_A_PIN, OUTPUT);
	pinMode(MOTOR_PWM_B_PIN, OUTPUT);
	pinMode(MOTOR_ENABLE_A_PIN, OUTPUT);
	pinMode(MOTOR_ENABLE_B_PIN, OUTPUT);
	// Note: A6 and A7 are analog-only, no pinMode needed for current sense

	// INA226 alert pin
	pinMode(INA226_ALERT_PIN, INPUT_PULLUP);
	// Attach Pin Change Interrupt to INA226 alert pin (pin 10)
	attachPCINT(digitalPinToPCINT(INA226_ALERT_PIN), ina226_interrupt, FALLING);

	// Encoder pins
	pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
	pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
	
	// Attach interrupt to encoder CLK pin
	attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoder_interrupt, CHANGE);

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
		Serial.println(F("ERROR: Failed to initialize INA226 at address 0x44"));
		Serial.println(F("Check wiring and connections"));
		init_failed = 1;
	} else {
		Serial.println(F("INA226 initialized successfully at 0x44"));
		ina226_initialized = true;
	}
	
	// Configure INA226 if it was successfully initialized
	if (ina226_initialized) {
		Serial.println(F("Configuring INA226..."));
		
		// Configure INA226 for 0.1 ohm shunt with LIMITED 0.8A max current
		float shunt_ohms = 0.1;  // 100 milliohm shunt (R100) - LIMITS RANGE!
		float max_current = 0.8;  // 0.8A max current (INA226 limit with 0.1Ω shunt)
		
		Serial.print(F("Calibrating with max current: "));
		Serial.print(max_current);
		Serial.print(F("A, shunt: "));
		Serial.print(shunt_ohms);
		Serial.print(F(" ohm... "));
		
		int cal_result = ina226.setMaxCurrentShunt(max_current, shunt_ohms, true);
		if (cal_result == 0) {
			Serial.println(F("SUCCESS!"));
			
			Serial.print(F("INA226 calibrated successfully. Current LSB: "));
			Serial.print(ina226.getCurrentLSB_mA());
			Serial.println(F(" mA"));
			Serial.print(F("IMPORTANT: Current measurement saturates at "));
			Serial.print(max_current);
			Serial.println(F("A with this shunt!"));
			
			// Set conversion time for both shunt and bus voltage (default is fine)
			// Enable continuous shunt and bus voltage monitoring
			ina226.setModeShuntBusContinuous();
			
			// Configure alert for overcurrent detection (0.7A threshold)
			// For 0.1 ohm shunt: 0.7A * 0.1Ω = 0.07V = 70mV
			// INA226 shunt voltage register LSB is 2.5µV, so 70mV = 70000µV / 2.5µV = 28000 counts
			uint16_t alert_limit = 28000;  // 0.7A * 0.1Ω / 2.5µV
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
			if (encoder_position >= encoder_target_position) {
				Serial.print(F("Motor stopped: up position limit reached at "));
				Serial.println(encoder_position);
				must_stop = true;
			}
		} else { // motor_direction == 'D'
			// Moving down - stop if we've reached or passed the target (going positive)
			if (encoder_position <= encoder_target_position) {
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
			
			// Read bus voltage (mV)
			float bus_voltage_v = ina226.getBusVoltage();
			uint16_t voltage_mv = (uint16_t)(bus_voltage_v * 1000.0);
			send_battery_voltage(voltage_mv);
			
			// Read signed current (mA) - positive = discharging, negative = charging
			float current_a = ina226.getCurrent();
			int16_t current_ma = (int16_t)(current_a * 1000.0);
			send_battery_current(current_ma);
			
			// Read power (mW) - always positive in INA226
			float power_w = ina226.getPower();
			long temp_power_mw = (long)(power_w * 1000.0);

			// Clamp to int16_t bounds
			if (temp_power_mw > INT16_MAX) temp_power_mw = INT16_MAX;
			if (temp_power_mw < INT16_MIN) temp_power_mw = INT16_MIN;

			int16_t power_mw = (int16_t)temp_power_mw;
		
			// Apply sign of current since power is unsigned in INA226
			power_mw = (current_a >= 0) ? power_mw : -power_mw; 
			send_battery_power((uint16_t)power_mw);
			
			Serial.print("INA226: ");
			Serial.print(bus_voltage_v);
			Serial.print(F("V, "));
			Serial.print(current_a);
			Serial.print(F("A"));
			Serial.print(F(", "));
			Serial.print(power_mw);
			Serial.println(F("mW"));
			
			// Put INA226 back to sleep if motor is not running
			if (was_sleeping) {
				ina226_sleep();
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
		} else {
			printf("Read %d thermometer(s)\n", thermometer_count);
		}
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



