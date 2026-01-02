#include <SPI.h>
#include <JeeLib.h>
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
 * Pin Assignment for Arduino Pro Mini (3.3V/8MHz) - Gate Opener:
 * 
 * Digital Pins:
 *   0  - RX (Serial) - Reserved for programming/debugging
 *   1  - TX (Serial) - Reserved for programming/debugging  
 *   2  - GATE_LED_OPENING detection (interrupt capable) - voltage divider from gate opener LED R1 = 680+100k, R2 = 220k
 *   3  - RF24 IRQ (interrupt capable) - for radio packet reception
 *   4  - GATE_LED_CLOSING detection - voltage divider from gate opener LED R1 = 680+100k, R2 = 220k
 *   5  - Gate OPEN control via MOSFET (HIGH=activate, LOW=inactive)
 *   6  - Gate CLOSE control via MOSFET (HIGH=activate, LOW=inactive)
 *   7  - Gate STOP control via MOSFET (HIGH=activate, LOW=inactive)
 *        Hardware: Each pin → 10kΩ → N-ch MOSFET gate (2N7002/BSS138)
 *                  Gate control line ← MOSFET drain, source → GND
 *   8  - RF24 CSN
 *   9  - RF24 CE
 *   10 - INA226 Alert pin (Pin Change Interrupt capable)
 *   11 - RF24 MOSI (SPI)
 *   12 - RF24 MISO (SPI)
 *   13 - RF24 SCK (SPI) + onboard LED
 * 
 * Analog Pins:
 *   A0 - Available
 *   A1 - Available
 *   A2 - Available
 *   A3 - Available
 *   A4 - I2C SDA (INA226)
 *   A5 - I2C SCL (INA226)
 *   A6 - Available
 *   A7 - Available
 */


const int CE_PIN = 9;          // RF24 CE
const int CSN_PIN = 8;         // RF24 CSN
const int RF24_IRQ_PIN = 3;    // RF24 IRQ pin for external interrupts

// Gate status detection pins (voltage dividers from gate opener LEDs)
// Hardware: Gate LED → 680Ω + 100kΩ → Pin → 220kΩ → GND
// Logic: HIGH = LED off (gate inactive), LOW = LED on (gate moving)
const int GATE_LED_OPENING = 2;  // Gate opening status detection
const int GATE_LED_CLOSING = 4;  // Gate closing status detection

// INA226 pins
const int SCL_PIN = A5;        // I2C for INA226
const int SDA_PIN = A4;        // I2C for INA226
const int INA226_ALERT_PIN = 10; // INA226 alert pin (Pin Change Interrupt)

// Gate control pins via N-channel MOSFETs (3.3V Arduino → 5V gate board)
// Hardware: Arduino pin → 10kΩ → MOSFET gate (2N7002/BSS138)
//          Gate line ← MOSFET drain, MOSFET source → GND
//          External 5V pullup on gate line
// Logic: Arduino HIGH → MOSFET ON → Gate line LOW (activate)
//        Arduino LOW → MOSFET OFF → Gate line HIGH via pullup (inactive)
const int GATE_OPEN_PIN = 5;   // Gate open control via MOSFET
const int GATE_CLOSE_PIN = 6;  // Gate close control via MOSFET
const int GATE_STOP_PIN = 7;   // Gate stop control via MOSFET


// INA226 setup
const uint8_t INA226_I2C_ADDRESS = 0x40;
INA226 ina226(INA226_I2C_ADDRESS); 

// INA226 alert handling
volatile bool ina226_alert_triggered = false;

// Gate status detection (from LED voltage dividers)
volatile bool gate_status_IRQ_triggered = false;
bool gate_opening = false;  // true when GATE_LED_OPENING is LOW
bool gate_closing = false;  // true when GATE_LED_CLOSING is LOW

// Gate control state
static bool gate_moving = false;
static char gate_action = 'S'; // 'O'=Open, 'C'=Close, 'S'=Stop

#if HAS_RF24
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe_address_gate = 0xF0F0F0F0F2LL;
#define PIPE_GATE 1
#endif

// Timing variables
static unsigned long send_next_status_at = 0; // Time for next status send
const unsigned long GATE_OPERATION_TIMEOUT = 30000; // 30 seconds max operation time

// Reboot mechanism to avoid millis() overflow issues
const unsigned long REBOOT_AT_MILLIS = 4200000000UL; // ~48.6 days

// RF24 interrupt handling
volatile bool radio_packet_received = false;

// Radio configuration monitoring
static unsigned long check_radio_config_at = 0;
static const unsigned long RADIO_CONFIG_CHECK_INTERVAL_IDLE = 3600000;  // 1 hour when idle


int init_failed = 0;
bool ina226_initialized = false;
		

// INA226 hardware constants
const float SHUNT_RESISTANCE_OHMS = 0.010f;       // 10 milliohm shunt (R010)
const float INA226_SHUNT_VOLTAGE_LSB = 0.0000025f; // 2.5µV per LSB for shunt voltage register

// RF24 interrupt handler
void rf24_interrupt() {
    radio_packet_received = true;
}

// INA226 interrupt handler
void ina226_interrupt() {
    // INA226 alert triggered - set flag for main loop processing
    ina226_alert_triggered = true;
}

// Gate opening status interrupt handler (pin 2)
void gate_moving_interrupt() {
    gate_status_IRQ_triggered = true;
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

// Helper function to activate a gate control pin momentarily
// pin: the pin to activate (GATE_OPEN_PIN, GATE_CLOSE_PIN, or GATE_STOP_PIN)
void activate_gate_pin(int pin)
{
	// Stop any current operation first - all MOSFETs OFF
	digitalWrite(GATE_OPEN_PIN, LOW);
	digitalWrite(GATE_CLOSE_PIN, LOW);
	digitalWrite(GATE_STOP_PIN, LOW);
	
	// Activate target pin via MOSFET (Arduino HIGH → MOSFET ON → Gate line LOW)
	digitalWrite(pin, HIGH);
	delay(500); // Brief pulse
	digitalWrite(pin, LOW); // MOSFET OFF → Gate line HIGH via pullup
}

void open_gate()
{
	activate_gate_pin(GATE_OPEN_PIN);
}

void close_gate()
{
	activate_gate_pin(GATE_CLOSE_PIN);
}

void stop_gate()
{
	activate_gate_pin(GATE_STOP_PIN);
}


int radio_send(uint8_t pipe_id, uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
#if HAS_RF24
	uint8_t payload[4] = { p0, p1, p2, p3 };
	
	// Must stop listening before changing writing pipe
	radio.stopListening();
	
	// Use gate pipe
	radio.openWritingPipe(pipe_address_gate);
	
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
		// Radio send failed - trigger immediate config checks
		check_radio_config_at = millis();
		return -1;
	}
#endif
	return 0;
}

void send_battery_voltage(uint16_t voltage_mv)
{
	radio_send(PIPE_GATE, 'V', voltage_mv & 0xFF, (voltage_mv >> 8) & 0xFF, 0);
}

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
    
    radio_send(PIPE_GATE, 'P', b0, b1, b2);
}

void send_battery_current(int16_t current_ma)
{
	radio_send(PIPE_GATE, 'I', current_ma & 0xFF, (current_ma >> 8) & 0xFF, 0);
}

void send_gate_status()
{
	check_gate_status();
	// 'Q' for status, gate_action is 'O'/'C'/'S'/'b' (booting)
	radio_send(PIPE_GATE, 'Q', gate_action, 0, 0); 
}

void process_gate_command(uint8_t p0, uint8_t cmd)
{
	Serial.print(F("Gate command: "));
	Serial.print((char)p0);
	Serial.print(F(" "));
	Serial.print((char)cmd);
	Serial.println();

	if (p0 != 'G') {
		return;
	}

	switch (cmd) {
		case 'O': // Open
			open_gate();
			break;
		case 'C': // Close
			close_gate();
			break;
		case 'S': // Stop
			stop_gate();
			break;
		case 'Q': // Query status
			// Fall through to send_next_status_at override below
			break;
	}
	send_next_status_at = millis();
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
		
		if (pipe_num == PIPE_GATE) {
			process_gate_command(payload[0], payload[1]);
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
#if HAS_RF24
void configure_radio()
{
	// Configure all radio settings
	radio.setRetries(15, 15);
	radio.setChannel(80);
	radio.setCRCLength(RF24_CRC_16);
	radio.setPayloadSize(4);
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(true);

	// Setup gate pipe for reading and writing
	radio.openWritingPipe(pipe_address_gate);
	radio.openReadingPipe(PIPE_GATE, pipe_address_gate);
	
	// Enable interrupt ONLY on data received (RX_DR)
	// maskIRQ(tx_ds, tx_fail, rx_ready) - 1=mask(disable), 0=enable
	radio.maskIRQ(1, 1, 0); // Mask TX_DS and MAX_RT, enable RX_DR only
	
	radio.startListening();
	radio.printDetails();
}

bool check_radio_configuration() {
	uint8_t radio_details[43];
	radio.encodeRadioDetails(radio_details);
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

	// Check RF_CH register (array index 5)
	if (radio_details[5] != 80) {
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
	
	// Check EN_RXADDR register (array index 2) - should enable pipe 1
	// 0x02 = ERX_P1 (bit 1)
	if ((radio_details[2] & 0x02) != 0x02) {
		Serial.println(F("Radio config mismatch on EN_RXADDR"));
		goto mismatch;
	}
	
	// Check RX_ADDR_P1 (array indices 15-19) - should match pipe_address_gate
	for (int i = 0; i < 5; i++) {
		uint8_t expected_byte = (pipe_address_gate >> (i * 8)) & 0xFF;
		if (radio_details[15 + i] != expected_byte) {
			Serial.println(F("Radio config mismatch on RX_ADDR_P1"));
			goto mismatch;
		}
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
	radio_send(PIPE_GATE, 'Q', 'b', 'r', 0);
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
			Serial.println(F(" - expected devID 0x226)"));
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
	}
}

// Handle gate status changes from LED monitoring
void check_gate_status() {
	// Determine current gate movement state from LED pins (active low)
	gate_opening = !digitalRead(GATE_LED_OPENING); 
	gate_closing = !digitalRead(GATE_LED_CLOSING);
	bool currently_moving = gate_opening || gate_closing;

	// Update gate action based on which LED is active
	if (gate_opening && !gate_closing) {
		gate_action = 'O'; // Opening
	} else if (gate_closing && !gate_opening) {
		gate_action = 'C'; // Closing  
	} else {
		gate_action = 'S'; // Stopped
	}
	
	// Handle state transitions
	if (currently_moving == gate_moving) {
		// No change of status
		return;
	}

	if (currently_moving) {
		// Gate started moving - wake up INA226 and enable fast reporting
		Serial.print(F("Gate "));
		if (gate_action == 'O') {
			Serial.print(F("opening"));
		} else if (gate_action == 'C') {
			Serial.print(F("closing"));
		}
		Serial.println(F(" started - enabling fast monitoring"));
		ina226_wake();
	} else {
		// Gate stopped moving - put INA226 to sleep and return to slow reporting
		Serial.println(F("Gate movement stopped"));
		ina226_sleep();
	}
		
	send_next_status_at = millis(); // Send status immediately
	gate_moving = currently_moving;
}

// Helper function to configure INA226 with calibration 
bool configure_ina226() {
	// Calculate max current based on INA226 library constraint
	// Library checks: maxCurrent * shunt <= 0.08190V (81.90mV)
	float max_current = 0.08190 / SHUNT_RESISTANCE_OHMS;  // Maximum current the library will accept
	
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
	Serial.print(F("Current measurement range up to "));
	Serial.print(max_current);
	Serial.print(F("A with "));
	Serial.print(SHUNT_RESISTANCE_OHMS);
	Serial.println(F("Ω shunt"));
	
	// Set conversion time for both shunt and bus voltage (default is fine)
	// Enable continuous shunt and bus voltage monitoring
	ina226.setModeShuntBusContinuous();
	
	return true;
}

void setup(){
	printf_begin();
	Serial.begin(115200);
	Serial.println(F("Gate opener controller starting..."));  

	// Gate control pins via MOSFETs - start with all MOSFETs OFF (gate lines HIGH)
	pinMode(GATE_OPEN_PIN, OUTPUT);
	digitalWrite(GATE_OPEN_PIN, LOW);  // MOSFET OFF → Gate line HIGH via pullup
	pinMode(GATE_CLOSE_PIN, OUTPUT);
	digitalWrite(GATE_CLOSE_PIN, LOW); // MOSFET OFF → Gate line HIGH via pullup
	pinMode(GATE_STOP_PIN, OUTPUT);
	digitalWrite(GATE_STOP_PIN, LOW);  // MOSFET OFF → Gate line HIGH via pullup

	// Gate status detection pins (no pullups - external voltage dividers provide logic levels)
	pinMode(GATE_LED_OPENING, INPUT);
	pinMode(GATE_LED_CLOSING, INPUT);
	
	// Attach interrupts to gate status pins for change detection
	attachInterrupt(digitalPinToInterrupt(GATE_LED_OPENING), gate_moving_interrupt, CHANGE);
	attachPCINT(digitalPinToPCINT(GATE_LED_CLOSING), gate_moving_interrupt, CHANGE);
	
	// Read initial states
	gate_opening = (digitalRead(GATE_LED_OPENING) == LOW);
	gate_closing = (digitalRead(GATE_LED_CLOSING) == LOW);

	// INA226 alert pin
	pinMode(INA226_ALERT_PIN, INPUT_PULLUP);
	// Attach Pin Change Interrupt to INA226 alert pin (pin 10)
	//attachPCINT(digitalPinToPCINT(INA226_ALERT_PIN), ina226_interrupt, FALLING);

	// RF24 IRQ pin
	pinMode(RF24_IRQ_PIN, INPUT_PULLUP); 
	attachInterrupt(digitalPinToInterrupt(RF24_IRQ_PIN), rf24_interrupt, FALLING);

#if HAS_RF24
	// Radio init
	radio.begin();
	radio.powerDown();
	
	// Apply all radio configuration settings
	configure_radio();

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
		
		// Check INA226 identification to verify authenticity
		check_ina226_identification();
	}
	
	// Configure INA226 if it was successfully initialized
	if (ina226_initialized) {
		Serial.println(F("Configuring INA226..."));
		
		if (!configure_ina226()) {
			Serial.println(F("ERROR: INA226 configuration failed"));
			ina226_initialized = false;
			init_failed = 1;
		} else {
			// Start with INA226 in sleep mode since gate should be stopped at startup
			ina226_sleep();
			Serial.println(F("INA226 initialized in sleep mode"));
		}
	}

	Serial.println(F("Gate opener controller ready"));

	radio_send(PIPE_GATE, 'Q', 'b', 0, 0); //"booting"
}

void loop() 
{
	// Handle gate status changes from LED monitoring
	if (gate_status_IRQ_triggered) {
		gate_status_IRQ_triggered = false; // Clear flag
		check_gate_status();
	}

	if (radio_packet_received) {
		radio_packet_received = false; // Clear flag
		check_radio_messages();
	}

	// Send status
	if (millis() >= send_next_status_at) {
		if (gate_moving) {
			send_next_status_at = millis() + 5000LL;
		} else {
			send_next_status_at = millis() + 60 * 60 * 1000LL; // 60 minutes when idle
		}
		send_gate_status();
		
		// Read and send INA226 measurements
		if (ina226_initialized) {
			bool was_sleeping = !gate_moving;
			
			// Wake up INA226 if it was sleeping for measurement (only when not moving)
			if (was_sleeping) {
				ina226_wake();
				delay(50); // Allow time for measurements to stabilize
			}
			
			float bus_voltage_v = ina226.getBusVoltage();
			uint16_t voltage_mv = (uint16_t)(bus_voltage_v * 1000.0);
			send_battery_voltage(voltage_mv);
			
			// Read signed current (mA) - positive = discharging, negative = charging
			float current_a = ina226.getCurrent();
			int16_t current_ma = (int16_t)(current_a * 1000.0);
			send_battery_current(current_ma);
			
			// Read power (mW) and apply voltage correction
			float power_w = ina226.getPower();
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
			Serial.print(power_mw);
			Serial.println(F("mW"));
			
			// Put INA226 back to sleep if gate is not moving
			if (was_sleeping) {
				ina226_sleep();
			}
		}
	}

	
	// Monitor and reset radio/INA226 configuration if corrupted by EMI
#if HAS_RF24
	if (millis() >= check_radio_config_at) {
		check_radio_config_at = millis() + RADIO_CONFIG_CHECK_INTERVAL_IDLE;
		
		if (!check_radio_configuration()) {
			reset_radio_configuration();
		}
	}
#endif

	// Check for millis() overflow prevention reboot
	if (millis() >= REBOOT_AT_MILLIS && !gate_moving) {
		software_reboot();
	}

	// Power management: calculate optimal sleep time
	if (!gate_moving && !radio_packet_received) {
		// Calculate time until next required action
		unsigned long current_time = millis();
		unsigned long sleep_time_status;
		if (current_time > send_next_status_at)	
			sleep_time_status = 0;
		else 
			sleep_time_status = send_next_status_at - current_time;

		unsigned long sleep_time = sleep_time_status;
		
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



