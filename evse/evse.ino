#include <ESP8266WiFi.h>
#include <PicoMQTT.h>
#include <ModbusMaster.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include "wifi_params.h"
#include "mqtt_login.h"
#include <ArduinoOTA.h>

#define REG_CURRENT_CONFIG 1000  // Configured current
#define REG_CONFIG        2005  // Configuration register

// MQTT topics
const char* MQTT_TOPIC_SET_CURRENT = "svevse/set_charge_current";
const char* MQTT_TOPIC_ENABLE = "svevse/enable_charge";
const char* MQTT_TOPIC_ADPS = "edf/ADPS";

// Global variables
ModbusMaster evse;
SoftwareSerial evseSerial(D1, D2);  // RX, TX
ESP8266WebServer server(80);
PicoMQTT::Client mqtt(MQTT_IP);

uint32_t adpsStopUntil = 0;
uint32_t next_publish_at = 0;
bool charging = false;

// HTML template
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="refresh" content="30">
<style>
  table { 
    border-collapse: collapse; 
    width: 100%; 
    margin: 20px 0;
  }
  th, td { 
    text-align: left; 
    padding: 12px; 
    border-bottom: 1px solid #ddd;
  }
  tr:nth-child(even) { 
    background-color: #f2f2f2;
  }
  th {
    background-color: #4CAF50;
    color: white;
  }
  .controls {
    margin: 20px 0;
  }
  button {
    padding: 10px 20px;
    margin-right: 10px;
    cursor: pointer;
  }
  input[type="number"] {
    padding: 8px;
    width: 80px;
  }
</style>
</head>
<body>
<h1>EVSE Status</h1>

<div class="controls">
  <button onclick="window.location.href='/start'">Start Charging</button>
  <button onclick="window.location.href='/stop'">Stop Charging</button>
  <form action="/set_current" method="post" style="display: inline">
    <input type="number" name="current" min="500" max="1600" value="%CURRENT%">
    <button type="submit">Set Current</button>
  </form>
<input type="button" value="Refresh" onClick="window.location.reload()">
</div>

<table>
  <tr>
    <th>Register</th>
    <th>Value</th>
  </tr>
  <tr>
    <td>1007 Status (relay)</td>
    <td>%RCD_STATUS%</td>
  </tr>
  <tr>
    <td>1000 Current Config</td>
    <td>%CURRENT_CONFIG% A</td>
  </tr>
  <tr>
    <td>1001 Current Output</td>
    <td>%CURRENT_OUTPUT% A</td>
  </tr>
  <tr>
    <td>1002 Vehicle State</td>
    <td>%VEHICLE_STATE%</td>
  </tr>
  <tr>
    <td>1003 PP Limit</td>
    <td>%PP_LIMIT% A</td>
  </tr>
  <tr>
    <td>1004 Control</td>
    <td>0x%CONTROL%</td>
  </tr>
  <tr>
    <td>1006 EVSE State</td>
    <td>%EVSE_STATE%</td>
  </tr>
  <tr>
    <td>2000 Max Current</td>
    <td>%MAX_CURRENT% A</td>
  </tr>
  <tr>
    <td>2005 Config</td>
    <td>0x%CONFIG%</td>
  </tr>
  <tr>
    <td>1005 Firmware</td>
    <td>%FIRMWARE%</td>
  </tr>
  <tr>
    <td>Registers 1008-1010</td>
    <td>0x%REG1008% 0x%REG1009% 0x%REG1010%</td>
  </tr>
  <tr>
    <td>2001 Modbus Address</td>
    <td>%MODBUS_ADDR%</td>
  </tr>
  <tr>
    <td>2002 Min Current</td>
    <td>%MIN_CURRENT% A</td>
  </tr>
  <tr>
    <td>2003 Analog Input</td>
    <td>0x%ANALOG_INPUT%</td>
  </tr>
  <tr>
    <td>2004 Power On</td>
    <td>0x%POWER_ON%</td>
  </tr>
  <tr>
    <td>2006 Sharing Mode</td>
    <td>%SHARING_MODE%</td>
  </tr>
  <tr>
    <td>2007 PP Detection</td>
    <td>%PP_DETECTION%</td>
  </tr>
  <tr>
    <td>Register 2008</td>
    <td>0x%REG2008%</td>
  </tr>
  <tr>
    <td>2009 Boot Firmware</td>
    <td>%BOOT_FW%</td>
  </tr>
</table>

</body>
</html>
)=====";

// Add register definitions
struct EvseRegisters {
    // 1000-series registers
    uint16_t currentConfig;    // 1000 - Configured current
    uint16_t currentOutput;    // 1001 - Actual output current
    uint16_t vehicleState;     // 1002 - Vehicle state
/*1: ready
2: EV is present
3: charging
4: charging with ventilation
5: failure (e.g. diode check, RCD failure)*/
    uint16_t ppLimit;         // 1003 - Cable PP limit
    uint16_t control;         // 1004 - Control register
    uint16_t firmware;        // 1005 - Firmware version
    uint16_t evseState;       // 1006 - EVSE state
    uint16_t rcdStatus;       // 1007 - status
    /*EVSE status and fails
bit0: relay on/off
bit1: diode check fail
bit2: vent required fail
bit3: waiting for pilot release (error recovery delay)
bit4: RCD check error
bit5:
bit6-bit15: reserved*/

    uint16_t reg1008;         // 1008
    uint16_t reg1009;         // 1009
    uint16_t reg1010;         // 1010

    // 2000-series registers
    uint16_t bootDefaultAmps;      // 2000 - Current after boot
    uint16_t modbusAddr;      // 2001 - Modbus address
    uint16_t minCurrent;      // 2002 - Minimum current
    uint16_t analogInput;     // 2003 - Analog input config
    uint16_t powerOn;         // 2004 - Power-on behavior
    uint16_t config;          // 2005 - Configuration register
    uint16_t sharingMode;     // 2006 - Sharing mode
    uint16_t ppDetection;     // 2007 - PP detection
    uint16_t reg2008;         // 2008
    uint16_t bootFirmware;    // 2009 - Boot firmware version
    uint16_t reg2010;         // 2010
    uint16_t reg2011;         // 2011
    uint16_t reg2012;         // 2012
    uint16_t reg2013;         // 2013
    uint16_t reg2014;         // 2014
    uint16_t reg2015;         // 2015
    uint16_t reg2016;         // 2016
    uint16_t reg2017;         // 2017
} evseRegs;

struct EvseRegisters oldEvseRegs;

bool readEvseRegisters() {
    uint8_t result;
    static uint8_t count = 0;
    static uint16_t last_sum = 0;
    
    // Read 1000-series registers (11 registers)
    result = evse.readHoldingRegisters(1000, 11);
    if (result != 0) {
        Serial.printf("Error reading registers 1000-1010: %d\n", result);
        return false;
    }

    oldEvseRegs = evseRegs;

    // Store 1000-series registers
    evseRegs.currentConfig = evse.getResponseBuffer(0);
    evseRegs.currentOutput = evse.getResponseBuffer(1);
    evseRegs.vehicleState = evse.getResponseBuffer(2);
    evseRegs.ppLimit = evse.getResponseBuffer(3);
    evseRegs.control = evse.getResponseBuffer(4);
    evseRegs.firmware = evse.getResponseBuffer(5);
    evseRegs.evseState = evse.getResponseBuffer(6);
    evseRegs.rcdStatus = evse.getResponseBuffer(7);
    evseRegs.reg1008 = evse.getResponseBuffer(8);
    evseRegs.reg1009 = evse.getResponseBuffer(9);
    evseRegs.reg1010 = evse.getResponseBuffer(10);

    // Read 2000-series registers (18 registers)
    result = evse.readHoldingRegisters(2000, 18);
    if (result != 0) {
        Serial.printf("Error reading registers 2000-2017: %d\n", result);
        return false;
    }

    // Store 2000-series registers
    evseRegs.bootDefaultAmps = evse.getResponseBuffer(0);
    evseRegs.modbusAddr = evse.getResponseBuffer(1);
    evseRegs.minCurrent = evse.getResponseBuffer(2);
    evseRegs.analogInput = evse.getResponseBuffer(3);
    evseRegs.powerOn = evse.getResponseBuffer(4);
    evseRegs.config = evse.getResponseBuffer(5);
    evseRegs.sharingMode = evse.getResponseBuffer(6);
    evseRegs.ppDetection = evse.getResponseBuffer(7);
    evseRegs.reg2008 = evse.getResponseBuffer(8);
    evseRegs.bootFirmware = evse.getResponseBuffer(9);
    evseRegs.reg2010 = evse.getResponseBuffer(10);
    evseRegs.reg2011 = evse.getResponseBuffer(11);
    evseRegs.reg2012 = evse.getResponseBuffer(12);
    evseRegs.reg2013 = evse.getResponseBuffer(13);
    evseRegs.reg2014 = evse.getResponseBuffer(14);
    evseRegs.reg2015 = evse.getResponseBuffer(15);
    evseRegs.reg2016 = evse.getResponseBuffer(16);
    evseRegs.reg2017 = evse.getResponseBuffer(17);

/*    if (memcmp(&evseRegs, &oldEvseRegs, sizeof(oldEvseRegs))) {
        next_publish_at = millis();
    }*/
    uint16_t sum = evseRegs.currentConfig + evseRegs.currentOutput + evseRegs.vehicleState + evseRegs.control + evseRegs.evseState + evseRegs.config;
    if (sum != last_sum) {
        // If "important" registers have changed values, report right away
        next_publish_at = millis();
        last_sum = sum;
    }

    // Print all registers to serial
    Serial.println("\nEVSE Registers:");
    Serial.printf("1000 Current Config: %f A\r\n", evseRegs.currentConfig / 100.);
    Serial.printf("1001 Current Output: %d A\r\n", evseRegs.currentOutput);
    Serial.printf("1002 Vehicle State: %d\r\n", evseRegs.vehicleState);
    Serial.printf("1004 Control: 0x%04X\r\n", evseRegs.control);
    Serial.printf("1006 EVSE State: %d\r\n", evseRegs.evseState);
    Serial.printf("2005 Config: 0x%04X\r\n", evseRegs.config);

    if (!(count++) % 10) {
        Serial.printf("1003 PP Limit: %d A\r\n", evseRegs.ppLimit);
        Serial.printf("1005 Firmware: %d\r\n", evseRegs.firmware);
        Serial.printf("1007 Status: %d\r\n", evseRegs.rcdStatus);
        Serial.printf("1008 Register: 0x%04X\r\n", evseRegs.reg1008);
        Serial.printf("1009 Register: 0x%04X\r\n", evseRegs.reg1009);
        Serial.printf("1010 Register: 0x%04X\r\n", evseRegs.reg1010);
        
        Serial.printf("2000 Max Current: %d A\r\n", evseRegs.bootDefaultAmps);
        Serial.printf("2001 Modbus Addr: %d\r\n", evseRegs.modbusAddr);
        Serial.printf("2002 Min Current: %d A\r\n", evseRegs.minCurrent);
        Serial.printf("2003 Analog Input: 0x%04X\r\n", evseRegs.analogInput);
        Serial.printf("2004 Power On: 0x%04X\r\n", evseRegs.powerOn);
        Serial.printf("2006 Sharing Mode: %d\r\n", evseRegs.sharingMode);
        Serial.printf("2007 PP Detection: %d\r\n", evseRegs.ppDetection);
        Serial.printf("2008 Register: 0x%04X\r\n", evseRegs.reg2008);
        Serial.printf("2009 Boot FW: %d\r\n", evseRegs.bootFirmware);
        Serial.printf("2010-2017 Registers: %d %d %d %d %d %d %d %d\r\n", 
            evseRegs.reg2010, evseRegs.reg2011, evseRegs.reg2012, evseRegs.reg2013,
            evseRegs.reg2014, evseRegs.reg2015, evseRegs.reg2016, evseRegs.reg2017);
    }

    return true;
}

void setupMQTT() {
    mqtt.username = MQTT_USER;
    mqtt.password = MQTT_PASS;
    
    mqtt.subscribe(MQTT_TOPIC_ENABLE, [](const char* payload) {
        if (strcmp(payload, "1") == 0) {
            startCharging();
        } else {
            stopCharging();
        }
    });
    
    mqtt.subscribe(MQTT_TOPIC_SET_CURRENT, [](const char* payload) {
        int current = 100 * atof(payload);
        setChargeCurrent(current);
    });
    
    mqtt.subscribe(MQTT_TOPIC_ADPS, [](const char* payload) {
        if (strcmp(payload, "1") == 0) {
            mqtt.publish("svevse/log", "ADPS, stopping charge");
            adpsStopUntil = millis() + 1200000; // 20 minutes
            stopCharging();
        }
    });

    mqtt.subscribe("edf/PTEC", [](const char *payload) {
            static uint8_t last_period_type = 0;
            char period = payload[0];

            if (period == last_period_type) {
                return;
            }

            last_period_type = period;

            /* PJB PJR PJW, CJB CJR CJW */
            if (period == 'C') {
                mqtt.publish("svevse/log", "Entering Heure Creuse, starting charge");
                startCharging();
            } else {
                mqtt.publish("svevse/log", "Leaving Heure Creuse, stopping charge");
                stopCharging();
            }
    });

    mqtt.begin();
}

void writeRegister(int reg, uint16_t val)
{
    evse.clearTransmitBuffer();
    evse.setTransmitBuffer(0, val);
    evse.writeMultipleRegisters(reg, 1);
}

void writeConfigBit(uint8_t bit)
{
    uint16_t config = 1 << 5 | 1 << 7 | 1 << 9 | 1 << bit;
    /*
       bit0: Enable button for current change (no sense when
2003 = 0)
0: disabled
1: enabled (default)
bit1: Stop charging when button pressed
0: disabled (default)
1: enabled
charging will automatically start after you manually unplug and
plug the cable to the vehicle
bit2: Pilot ready state LED
0: blinks once (default)
1: is always ON
bit3: enable charging on vehicle status D (ventilation required)
0: vehicle status D charging is disabled
1: vehicle status D charging is enabled (default)
bit4: enable RCD feedback on MCLR pin (pin 4)
0: disabled, no RCD connected (default)
1: enabled
bit5: auto clear RCD error
0: disabled (default, power cycle needed)
1: enabled (clear RCD error after <30s min timeout)
bit6: AN pullup (rev16 and later)
0: AN internal pull-up enabled (default)
1: AN internal pull-up disabled
bit7: PWM debug bit (rev17 and later)
0: PWM debug disabled (default)
1: PWM debug enable
bit8: error LED routing to AN out (rev17 and later)
0: disabled (default)
1: enabled
bit9: pilot auto recover delay (rev17 and later)
0: disabled
1: enabled (default)
bit10-11: reserved
bit12: enable startup delay
bit13: disable EVSE after charge (write 8192)
bit14: disable EVSE (write 16384)
bit15: enable bootloader mode (write 32768)*/
    writeRegister(REG_CONFIG, config);
    next_publish_at = millis() + 500;
}

bool startCharging() {
    if (millis() < adpsStopUntil) {
        return false;
    }

    writeRegister(1004, 0);
    writeConfigBit(13);
    charging = true;
    Serial.println("Charging started");
    return true;
}

bool stopCharging() {

/*    bit0: turn off charging now
bit1: run selftest and RCD test procedure (approx 30s)
bit2: clear RCD error
bit3 - bit15: not used*/
    writeRegister(1004, 1);

    writeConfigBit(14);
    charging = false;
    Serial.println("Charging stopped");
    return true;
}

bool setChargeCurrent(uint16_t current) {
    Serial.printf("Trying to set current to %d\r\n", current);
    writeRegister(REG_CURRENT_CONFIG, current);
    next_publish_at = millis() + 500;
    Serial.printf("Current set to %dA\n", current);
    return true;
}


void setupWebServer() {
    // Serve main page
    server.on("/", HTTP_GET, []() {
        String html = FPSTR(INDEX_HTML);
            html.replace("%CURRENT%", String(evseRegs.currentConfig));
        html.replace("%CURRENT_CONFIG%", String(evseRegs.currentConfig / 100.));
        html.replace("%CURRENT_OUTPUT%", String(evseRegs.currentOutput / 100.));
        html.replace("%VEHICLE_STATE%", String(evseRegs.vehicleState));
        html.replace("%PP_LIMIT%", String(evseRegs.ppLimit));
        html.replace("%CONTROL%", String(evseRegs.control, HEX));
        html.replace("%FIRMWARE%", String(evseRegs.firmware));
        html.replace("%EVSE_STATE%", String(evseRegs.evseState));
        html.replace("%RCD_STATUS%", String(evseRegs.rcdStatus));
        html.replace("%REG1008%", String(evseRegs.reg1008, HEX));
        html.replace("%REG1009%", String(evseRegs.reg1009, HEX));
        html.replace("%REG1010%", String(evseRegs.reg1010, HEX));
        html.replace("%MAX_CURRENT%", String(evseRegs.bootDefaultAmps));
        html.replace("%MODBUS_ADDR%", String(evseRegs.modbusAddr));
        html.replace("%MIN_CURRENT%", String(evseRegs.minCurrent));
        html.replace("%ANALOG_INPUT%", String(evseRegs.analogInput, HEX));
        html.replace("%POWER_ON%", String(evseRegs.powerOn, HEX));
        html.replace("%CONFIG%", String(evseRegs.config, HEX));
        html.replace("%SHARING_MODE%", String(evseRegs.sharingMode));
        html.replace("%PP_DETECTION%", String(evseRegs.ppDetection));
        html.replace("%REG2008%", String(evseRegs.reg2008, HEX));
        html.replace("%BOOT_FW%", String(evseRegs.bootFirmware));
        server.send(200, "text/html", html);
    });

    // JSON endpoint for register values
    server.on("/status", HTTP_GET, []() {
        String json = "{";
        json += "\"currentConfig\":" + String(evseRegs.currentConfig) + ",";
        json += "\"currentOutput\":" + String(evseRegs.currentOutput) + ",";
        json += "\"vehicleState\":" + String(evseRegs.vehicleState) + ",";
        json += "\"ppLimit\":" + String(evseRegs.ppLimit) + ",";
        json += "\"control\":" + String(evseRegs.control) + ",";
        json += "\"firmware\":" + String(evseRegs.firmware) + ",";
        json += "\"evseState\":" + String(evseRegs.evseState) + ",";
        json += "\"rcdStatus\":" + String(evseRegs.rcdStatus) + ",";
        json += "\"reg1008\":" + String(evseRegs.reg1008) + ",";
        json += "\"reg1009\":" + String(evseRegs.reg1009) + ",";
        json += "\"reg1010\":" + String(evseRegs.reg1010) + ",";
        // 2000 series registers
        json += "\"bootDefaultAmps\":" + String(evseRegs.bootDefaultAmps) + ",";
        json += "\"modbusAddr\":" + String(evseRegs.modbusAddr) + ",";
        json += "\"minCurrent\":" + String(evseRegs.minCurrent) + ",";
        json += "\"analogInput\":" + String(evseRegs.analogInput) + ",";
        json += "\"powerOn\":" + String(evseRegs.powerOn) + ",";
        json += "\"config\":" + String(evseRegs.config) + ",";
        json += "\"sharingMode\":" + String(evseRegs.sharingMode) + ",";
        json += "\"ppDetection\":" + String(evseRegs.ppDetection) + ",";
        json += "\"reg2008\":" + String(evseRegs.reg2008) + ",";
        json += "\"bootFirmware\":" + String(evseRegs.bootFirmware) + ",";
        json += "\"reg2010\":" + String(evseRegs.reg2010) + ",";
        json += "\"reg2011\":" + String(evseRegs.reg2011) + ",";
        json += "\"reg2012\":" + String(evseRegs.reg2012) + ",";
        json += "\"reg2013\":" + String(evseRegs.reg2013) + ",";
        json += "\"reg2014\":" + String(evseRegs.reg2014) + ",";
        json += "\"reg2015\":" + String(evseRegs.reg2015) + ",";
        json += "\"reg2016\":" + String(evseRegs.reg2016) + ",";
        json += "\"reg2017\":" + String(evseRegs.reg2017);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/start", HTTP_GET, []() {
        startCharging();
        server.sendHeader("Location", "/");
        server.send(303);
    });

    server.on("/stop", HTTP_GET, []() {
        stopCharging();
        server.sendHeader("Location", "/");
        server.send(303);
    });

    server.on("/set_current", HTTP_POST, []() {
        if (server.hasArg("current")) {
            uint16_t current = server.arg("current").toInt();
            setChargeCurrent(current);
        }
        server.sendHeader("Location", "/");
        server.send(303);
    });

    server.begin();
}

void setupOTA() {
    // Port defaults to 8266
    ArduinoOTA.setPort(8266);
    
    // Set hostname for easy identification
    ArduinoOTA.setHostname("SVEVSE");
    
    // Set password for OTA updates
    ArduinoOTA.setPassword("evse123");  // Change this password!

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("Starting OTA update (" + type + ")");
        
        // Stop MQTT and other communications during update
        mqtt.disconnect();
        evseSerial.end();
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA update complete");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA ready");
}

void setup() {
    Serial.begin(115200);
    evseSerial.begin(9600);
    evse.begin(1, evseSerial);
   
    Serial.println("Hello from Svenvse");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    setupOTA();
    setupMQTT();
    setupWebServer();
}

void publishStatus() {
    mqtt.publish("svevse/vehicle_state", String(evseRegs.vehicleState));
    mqtt.publish("svevse/current_output", String(evseRegs.currentOutput / 100.));
    mqtt.publish("svevse/evse_state", String(evseRegs.evseState));
    mqtt.publish("svevse/charging", charging ? "1" : "0");

    String statusString = "VehicleState ";
    switch (evseRegs.vehicleState) {
        case 1: 
            statusString += "ready";
            break;
        case 2:
            statusString += "present";
            break;
        case 3:
            statusString += "charging";
            break;
        case 4:
            statusString += "chargingWithVentilation";
            break;
        case 5:
            statusString += "error";
            break;
        default:
            statusString += "UNKNOWN VALUE " + String(evseRegs.vehicleState);
    }

    statusString += " - EVSEState ";
    switch (evseRegs.evseState) {
        case 1:
            statusString += "Steady12V";
            break;
            case 2:
            statusString += "PWM";
            break;
            case 3:
            statusString += "OFF";
            break;

        default:
            statusString += "UNKNOWN VALUE " + String(evseRegs.evseState);
    }
    
    statusString += " - RelayState " + String(evseRegs.rcdStatus & 1);
    mqtt.publish("svevse/status_string", statusString);

    // Default safety values: boot current 8A, max cable 10A
    writeRegister(2000, 8);
    writeRegister(2007, 10);

    // Minimal current for this car is 5A
    writeRegister(2002, 5);
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    server.handleClient();
   
    static uint32_t last_read_regs = 0;

    if (!last_read_regs || millis() - last_read_regs > 250) {
        readEvseRegisters();
        last_read_regs = millis();
    }

    // Regular status updates
    if (millis() > next_publish_at) {
        publishStatus();
        next_publish_at = millis() + 3 * 60000;
    }
    
    // Check ADPS timeout
    if (adpsStopUntil > 0 && millis() > adpsStopUntil) {
        adpsStopUntil = 0;
        startCharging();
    }
}

