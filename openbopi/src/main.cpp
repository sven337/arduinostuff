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

#include <DHT.h>

// Pumps
const int pump_pins[] = { 32, 33 };
unsigned long pump_stop_at[2];
bool pump_running[2];


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
float boxTemp = 0;
float boxHumidity = 0;

float orp_target = 750.0;  // Default ORP target
bool orp_regulation_enabled = false;
unsigned long last_chlorine_injection = 0;
const unsigned long MIN_INJECTION_INTERVAL = 300000;  // 5 minutes
const unsigned long CHLORINE_INJECTION_TIME = 10;   // 10 seconds injection
    
unsigned long next_publish_at = 0;
unsigned long next_read_data_at = 0;

#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>OpenBopi Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
        <meta charset="utf-8">
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
        <p>ORP Regulation: <span id="orp_regulation">%ORP_REGULATION%</span></p>
        <p>ORP Target: <input type="number" id="orp_target" value="%ORP_TARGET%" onchange="updateORPTarget(this.value)"> mV</p>
    </div>
    <button class="button" onclick="toggleORPRegulation()">Toggle ORP Regulation</button>
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
        }, 5000);
         function updateORPTarget(value) {
            fetch('/orp_target?value=' + value)
                .then(response => response.text())
                .then(data => console.log(data));
        }
        function toggleORPRegulation() {
            fetch('/toggle_regulation')
                .then(response => response.text())
                .then(data => console.log(data));
        }
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
}

void publishStatus() {
    String status = "{\"ph\":" + String(phValue, 2) + 
                   ",\"orp\":" + String(orpValue, 0) +
                  ",\"watertemp\":" + String(waterTemp, 1) + 
                   ",\"airtemp\":" + String(airTemp, 1) +
                   ",\"pump1\":" + String(pump_running[0]) +
                   ",\"pump2\":" + String(pump_running[1]) +
                   ",\"orp_regulation\":" + String(orp_regulation_enabled) +
                   ",\"orp_target\":" + String(orp_target) + "}";
    mqtt.publish("openbopi/pH", String(phValue, 2));
    mqtt.publish("openbopi/ORP", String(orpValue, 0));
    mqtt.publish("openbopi/orp_target", String(orp_target, 0));
    mqtt.publish("openbopi/orp_regulation", String(orp_regulation_enabled));
    mqtt.publish("openbopi/pump1", String(pump_running[0]));
    mqtt.publish("openbopi/pump2", String(pump_running[1]));
    mqtt.publish("openbopi/watertemp", String(waterTemp, 1));
    mqtt.publish("openbopi/airtemp", String(airTemp, 1));
    mqtt.publish("openbopi/boxtemp", String(boxTemp, 1));
    mqtt.publish("openbopi/boxhumidity", String(boxHumidity, 0));
    Serial.println(status);
}

void read_pH_ORP() {
    // Read pH from ADS1115 A2
    int16_t adc2 = ads.readADC_SingleEnded(2);
    int16_t adc3 = ads.readADC_SingleEnded(3);
    float v2 = ads.computeVolts(adc2); 
    float v3 = ads.computeVolts(adc3); 
    phValue = 7 + (1.53 - v2) / 0.14; // wtf but that seems to be what bopi does. channel 3 is ignored
    
    // Read ORP from ADS1115 A0/1
    int16_t adc0 = ads.readADC_Differential_0_1();
    orpValue = ads.computeVolts(adc0) * 1000.0;
}

void readTemperatures() {
    tempSensors.requestTemperatures();
    waterTemp = tempSensors.getTempCByIndex(0); // First sensor is water temp
    airTemp = tempSensors.getTempCByIndex(1);   // Second sensor is air temp
    
    // Check for valid readings
    if (waterTemp == DEVICE_DISCONNECTED_C) {
        waterTemp = -127;
    }
    if (airTemp == DEVICE_DISCONNECTED_C) {
        airTemp = -127;
    }
}

void readDHT11() 
{
    float newTemp = dht.readTemperature();
    float newHumidity = dht.readHumidity();
    
    // Only update if readings are valid
    if (!isnan(newTemp)) {
        boxTemp = newTemp;
    }
    if (!isnan(newHumidity)) {
        boxHumidity = newHumidity;
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

void start_pump(unsigned int index, int duration)
{
    if (index > 1) {
        return;
    }

    if (duration > 300) {
        return;
    }

    pump_stop_at[index] = millis() + duration * 1000;
    pump_running[index] = true;

    digitalWrite(pump_pins[index], HIGH);
    next_publish_at = millis();

}

void stop_pump(unsigned int index)
{
    if (index > 1) {
        return;
    }

    pump_stop_at[index] = 0;
    pump_running[index] = false;
    digitalWrite(pump_pins[index], LOW);
    next_publish_at = millis();
}

void checkPumps() {
    for (unsigned int i = 0; i < 2; i++) {
        if (!pump_running[i]) 
            continue;

        if (millis() > pump_stop_at[i]) {
            stop_pump(i);
        }
    }
}

void mqtt_show_status_cb(const char *payload) 
{
    next_publish_at = millis();
}

void mqtt_force_pump_cb(const char *payload) 
{
    int pumpId = atoi(payload);
    start_pump(pumpId, 10);
}

void mqtt_set_orp_target_cb(const char *payload) 
{
    orp_target = atof(payload);
    next_publish_at = millis();
}


void toggle_ORP_regulation(bool status)
{
    if (status == orp_regulation_enabled) {
        return;
    }

    orp_regulation_enabled = status;
    next_publish_at = millis();
}

void mqtt_enable_orp_regulation_cb(const char *payload) 
{
    toggle_ORP_regulation(atoi(payload));
}

void checkORPRegulation() {
    if (!orp_regulation_enabled) 
        return;
    
    if (phValue > 7.5) {
        mqtt.publish("openbopi/status", "pH too high for chlorine injection: suspected problem");
        stop_pump(1);
        return;
    }

    if (orpValue < 400) {
        mqtt.publish("openbopi/status", "ORP abnormally low");
    }
    
    unsigned long now = millis();
    if (orpValue < orp_target && 
        !pump_running[1] &&
        (now - last_chlorine_injection) > MIN_INJECTION_INTERVAL) {
       
        last_chlorine_injection = now;
        start_pump(1, CHLORINE_INJECTION_TIME);
        mqtt.publish("openbopi/status", "Injecting chlorine");
    }
}

void setupWebServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = String(index_html);
        html.replace("%PH%", String(phValue, 2));
        html.replace("%ORP%", String(orpValue, 0));
        html.replace("%WATERTEMP%", String(waterTemp, 1));
        html.replace("%AIRTEMP%", String(airTemp, 1));
        html.replace("%ORP_REGULATION%", String(orp_regulation_enabled));
        html.replace("%ORP_TARGET%", String(orp_target, 3));
        request->send(200, "text/html", html);
    });

    server.on("/pump", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("id")) {
            int pumpId = request->getParam("id")->value().toInt();
            start_pump(pumpId - 1, 10);
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/values", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"ph\":" + String(phValue, 2) + 
                     ",\"orp\":" + String(orpValue, 0) + 
                     ",\"watertemp\":" + String(waterTemp, 1) + 
                     ",\"airtemp\":" + String(airTemp, 1) + 
                     "}";
        request->send(200, "application/json", json);
    });

     server.on("/orp_target", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            orp_target = request->getParam("value")->value().toFloat();
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/toggle_regulation", HTTP_GET, [](AsyncWebServerRequest *request) {
        toggle_ORP_regulation(!orp_regulation_enabled);
        request->send(200, "text/plain", orp_regulation_enabled ? "ON" : "OFF");
    });

    server.begin();
}


void setup() {
    Serial.begin(115200);
    
    pinMode(pump_pins[0], OUTPUT);
    pinMode(pump_pins[1], OUTPUT);
    digitalWrite(pump_pins[0], LOW);
    digitalWrite(pump_pins[1], LOW);
   
    dht.begin();
    setupWiFi();
    setupOTA();
    setupADC();
    setupTempSensors();
   
    mqtt.subscribe("openbopi/show_status", &mqtt_show_status_cb);
    mqtt.subscribe("openbopi/force_pump_10sec", &mqtt_force_pump_cb);
    mqtt.subscribe("openbopi/set_orp_target", &mqtt_set_orp_target_cb);
    mqtt.subscribe("openbopi/enable_orp_regulation", &mqtt_enable_orp_regulation_cb);

    mqtt.begin();
    mqtt.loop();
    setupWebServer();
    Serial.println("Openbopi hello");
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
   
    if (millis() > next_read_data_at) {
        read_pH_ORP();
        readTemperatures();
        readDHT11();
        next_read_data_at = millis() + 5000;
    }

    if (millis() > next_publish_at) {
        publishStatus();
        next_publish_at = millis() + 5 * 60 * 1000;
    }
    
    checkORPRegulation();
    checkPumps();
}

