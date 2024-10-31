#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESPAsyncWebServer.h>
#include "wifi_params.h"
#include "mqtt_login.h"

#include <PicoMQTT.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions
const int PUMP1_PIN = 32;
const int PUMP2_PIN = 33;

// Global objects
Adafruit_ADS1115 ads;
AsyncWebServer server(80);
PicoMQTT::Client mqtt(MQTT_IP, 1883, "openbopi", MQTT_USER, MQTT_PASS);
OneWire oneWire(19);
DallasTemperature tempSensors(&oneWire);

// Sensor values
float phValue = 0;
float orpValue = 0;
float waterTemp = 0;
float airTemp = 0;

// Pump control variables
unsigned long pump1StartTime = 0;
unsigned long pump2StartTime = 0;
bool pump1Running = false;
bool pump2Running = false;

// HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>OpenBopi Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; }
        .button { background-color: #4CAF50; color: white; padding: 10px 20px; 
                 border: none; border-radius: 4px; margin: 5px; }
        .readings { font-size: 1.2em; margin: 20px; }
    </style>
</head>
<body>
    <h2>OpenBopi Control Panel</h2>
    <div class="readings">
        <p>pH Value: <span id="ph">%PH%</span></p>
        <p>ORP Value: <span id="orp">%ORP%</span> mV</p>
        <p>Water Temperature: <span id="watertemp">%WATERTEMP%</span>°C</p>
        <p>Air Temperature: <span id="airtemp">%AIRTEMP%</span>°C</p>
    </div>
    <button class="button" onclick="activatePump(1)">Activate Pump 1 (10s)</button>
    <button class="button" onclick="activatePump(2)">Activate Pump 2 (10s)</button>
    <script>
        function activatePump(pump) {
            fetch('/pump?id=' + pump)
                .then(response => response.text())
                .then(data => console.log(data));
        }
        setInterval(function() {
            fetch('/values')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('ph').innerHTML = data.ph;
                    document.getElementById('orp').innerHTML = data.orp;
                    document.getElementById('watertemp').innerHTML = data.watertemp;
                    document.getElementById('airtemp').innerHTML = data.airtemp;
                });
        }, 2000);
    </script>
</body>
</html>
)rawliteral";

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

void setupOTA() {
    ArduinoOTA.onStart([]() {
        Serial.println("Starting OTA update");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA update complete");
    });
    ArduinoOTA.begin();
}

void setupADC() {
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS1115");
    }
//    ads.setGain(GAIN_ONE);
}

void publishStatus() {
    String status = "{\"ph\":" + String(phValue, 2) + 
                   ",\"orp\":" + String(orpValue, 2) +
                  ",\"watertemp\":" + String(waterTemp, 2) + 
                   ",\"airtemp\":" + String(airTemp, 2) +
                   ",\"pump1\":" + String(pump1Running) +
                   ",\"pump2\":" + String(pump2Running) + "}";
    mqtt.publish("openbopi/pH", String(phValue, 2));
    mqtt.publish("openbopi/ORP", String(orpValue, 2));
    mqtt.publish("openbopi/pump1", String(pump1Running));
    mqtt.publish("openbopi/pump2", String(pump2Running));
    mqtt.publish("openbopi/watertemp", String(waterTemp, 2));
    mqtt.publish("openbopi/airtemp", String(airTemp, 2));
    Serial.println(status);
}

void checkPumps() {
    if (pump1Running && (millis() - pump1StartTime >= 10000)) {
        digitalWrite(PUMP1_PIN, LOW);
        pump1Running = false;
    }
    if (pump2Running && (millis() - pump2StartTime >= 10000)) {
        digitalWrite(PUMP2_PIN, LOW);
        pump2Running = false;
    }
}

void setupWebServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = String(index_html);
        html.replace("%PH%", String(phValue, 2));
        html.replace("%ORP%", String(orpValue, 2));
        html.replace("%WATERTEMP%", String(waterTemp, 2));
        html.replace("%AIRTEMP%", String(airTemp, 2));
        request->send(200, "text/html", html);
    });

    server.on("/pump", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("id")) {
            int pumpId = request->getParam("id")->value().toInt();
            if (pumpId == 1 && !pump1Running) {
                digitalWrite(PUMP1_PIN, HIGH);
                pump1Running = true;
                pump1StartTime = millis();
            } else if (pumpId == 2 && !pump2Running) {
                digitalWrite(PUMP2_PIN, HIGH);
                pump2Running = true;
                pump2StartTime = millis();
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/values", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"ph\":" + String(phValue, 2) + 
                     ",\"orp\":" + String(orpValue, 2) + "}";
        request->send(200, "application/json", json);
    });

    server.begin();
}

void read_pH_ORP() {
    // Read pH from ADS1115 A0
    int16_t adc2 = ads.readADC_SingleEnded(2);
    int16_t adc3 = ads.readADC_SingleEnded(3);
    float v2 = ads.computeVolts(adc2); 
    float v3 = ads.computeVolts(adc3); 
    phValue = (v2 - v3) * 1000.0;
   // * 3.5; // Convert to pH (adjust multiplier as needed)
    
    // Read ORP from ADS1115 A1
    int16_t adc0 = ads.readADC_Differential_0_1();
    orpValue = ads.computeVolts(adc0) * 1000.0;
}

void readTemperatures() {
    tempSensors.requestTemperatures();
    waterTemp = tempSensors.getTempCByIndex(0); // First sensor is water temp
    airTemp = tempSensors.getTempCByIndex(1);   // Second sensor is air temp
    
    // Check for valid readings
    if (waterTemp == DEVICE_DISCONNECTED_C) {
        Serial.println("Error reading water temperature sensor");
        waterTemp = -127;
    }
    if (airTemp == DEVICE_DISCONNECTED_C) {
        Serial.println("Error reading air temperature sensor");
        airTemp = -127;
    }
}

void setupTempSensors() {
    tempSensors.begin();
    
    // Print the number of temperature sensors found
    int deviceCount = tempSensors.getDeviceCount();
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" temperature sensors.");
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PUMP1_PIN, OUTPUT);
    pinMode(PUMP2_PIN, OUTPUT);
    digitalWrite(PUMP1_PIN, LOW);
    digitalWrite(PUMP2_PIN, LOW);
    
    setupWiFi();
    setupOTA();
    setupADC();
    setupTempSensors();
    
    mqtt.begin();
    mqtt.loop();
    setupWebServer();
    Serial.println("Openbopi hello");
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    
    static unsigned long lastRead = 0;
    if (millis() - lastRead >= 1000) {
        read_pH_ORP();
        readTemperatures();
        publishStatus();
        lastRead = millis();
    }
    
    checkPumps();
}

