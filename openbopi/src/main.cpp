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
#include <ESPmDNS.h>

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

float orp_target = 600.0;  // Default ORP target
bool orp_regulation_enabled = false;
unsigned long last_chlorine_injection = 0;
unsigned long min_injection_interval = 5 * 60 * 1000;  // 5 minutes
unsigned long chlorine_injection_time = 10;   // 10 seconds injection
    
unsigned long next_publish_at = 0;
unsigned long next_read_data_at = 0;

unsigned long fast_publish_until = 0;
unsigned long streaming_enabled_until = 0;

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
        .button { 
            background-color: #4CAF50; 
            color: white; 
            padding: 10px 20px; 
            border: none; 
            border-radius: 4px; 
            margin: 5px;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        .button:hover {
            background-color: #45a049;
            transform: translateY(-1px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        }
        .button:active {
            background-color: #3d8b40;
            transform: translateY(1px);
            box-shadow: 0 1px 2px rgba(0,0,0,0.2);
        }
        .readings { font-size: 1.2em; margin: 20px; }
        .status {
            display: inline-block;
            padding: 3px 8px;
            border-radius: 3px;
            font-weight: bold;
        }
        .status-on {
            background-color: #4CAF50;
            color: white;
        }
        .status-off {
            background-color: #f44336;
            color: white;
        }
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
        <p>Injection Time: <input type="number" id="injection_time" value="%INJECTION_TIME%" onchange="updateInjectionTime(this.value)"> seconds</p>
        <p>Injection Interval: <input type="number" id="injection_interval" value="%INJECTION_INTERVAL%" onchange="updateInjectionInterval(this.value)"> seconds</p>
        <p>Fast Publishing: <span id="fast_publish">%FAST_PUBLISH%</span></p>
        <p>Streaming Mode: <span id="streaming">%STREAMING%</span></p>
    </div>
     <button class="button" onclick="handleButton('/toggle_regulation')">Toggle ORP Regulation</button>
    <button class="button" onclick="handleButton('/pump?id=1')">Activate Pump 1 (10s)</button>
    <button class="button" onclick="handleButton('/pump?id=2')">Activate Pump 2 (10s)</button>
    <button class="button" onclick="handleButton('/fast_publish')">Enable Fast Publishing (60min)</button>
    <button class="button" onclick="handleButton('/streaming')">Enable Streaming (10min)</button>
    <script>
        function handleButton(url) {
            fetch(url)
                .then(response => response.text())
                .then(data => {
                    console.log(data);
                    setTimeout(() => window.location.reload(), 500);
                })
                .catch(error => console.error('Error:', error));
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
        function updateInjectionTime(value) {
            fetch('/injection_time?value=' + value)
                .then(response => response.text())
                .then(data => console.log(data));
        }
        function updateInjectionInterval(value) {
            fetch('/injection_interval?value=' + value)
                .then(response => response.text())
                .then(data => console.log(data));
        }
    </script>
</body>
</html>
)rawliteral";

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("Disconnected from WiFi access point");
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.wifi_sta_disconnected.reason);
    WiFi.begin(ssid, password);
}

void setupWiFi() {
    WiFi.setHostname("openbopi");
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
    WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
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
                   ",\"orp_target\":" + String(orp_target) + 
                   ",\"injection_time\":" + String(chlorine_injection_time) +
                   ",\"injection_interval\":" + String(min_injection_interval / 1000) + "}";
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
    mqtt.publish("openbopi/injection_time", String(chlorine_injection_time));
    mqtt.publish("openbopi/injection_interval", String(min_injection_interval / 1000));
    Serial.println(status);
}

void read_pH_ORP() {
    // Read pH from ADS1115 A2
    int16_t adc2 = ads.readADC_SingleEnded(2);
    int16_t adc3 = ads.readADC_SingleEnded(3);
    float v2 = ads.computeVolts(adc2); 
    float v3 = ads.computeVolts(adc3); 
    float newPhValue = 7 + (1.53 - v2) / 0.14; // wtf but that seems to be what bopi does. channel 3 is ignored
    // Filter with EMA 10%, see below for details
    if (phValue == 0) {
        phValue = newPhValue;
    } else {
        // pH EMA is 1/30th to try to and dampen the noise
        phValue = (29 * phValue + newPhValue) / 30;
    }
    
    // Read ORP from ADS1115 A0/1
    int16_t adc0 = ads.readADC_Differential_0_1();
    float newOrpValue = ads.computeVolts(adc0) * 1000.0;

    // Temperature-compensate ORP?
    // The slope of my ORP probe is -1.37mV/°C
    // This is significant enough that the setpoint at 15°C and 25°C will
    // correspond to differing amounts of chlorine, but what is interesting is
    // that the negative slope means that at lower temperature (where we need
    // /less/ sanitizer in the water), the ORP value will appear artificially
    // higher.
    // Do NOT compensate for temperature for that reason: yes, the ORP value
    // will appear artificially high at low temps, and artificially low at high
    // temps, but this is a direction of events which matches what we want to do
    // anyway (= put more chlorine when the water is hot and there are people
    // using the pool).

    // newOrpValue += 1.37 * (25 - waterTemp);

    // Filter ORP?
    // PoolMaster averages the 5 middle values in a buffer of 10, but takes 8
    // samples per second.
    // https://github.com/Gixy31/ESP32-PoolMaster/blob/ce8bc6853e5e7f942b50a10a78e9c2564d3d9e38/src/Loops.cpp#L36
    // I haven't seen a mathematical justification for it.
    // https://forum.arduino.cc/t/poolmaster-gestion-et-domotisation-de-ma-piscine/563058/895
    //
    //
    // BoPi averages the 18 middle values in a buffer of 20, I do not know the
    // exact sample rate (once per loop call?).
    //
    // In both cases this seems way higher than we need, since the swimming pool
    // is a very large body of water in which we're diluting our chlorine,
    // meaning that it will dampen variations dramatically. It seems unlikely
    // that the chlorine will take less than 5 minutes to fully dilute into the
    // water, so I do not naturally see why a higher rate of sampling is
    // helpful.
    //
    // This also implies that a tight regulation loop isn't needed - just inject
    // when you need more at a low duty cycle and you should be good. PID seems
    // overkill.
    //
    // In my filtering evaluations (see the img/ folder), an exponential moving
    // average actually works better than a median filter (as implemented by
    // PoolMaster) or a winsorized means (as implemented by Bopi). These were
    // done with 25 measurements/sec (streaming as fast as possible).
    if (orpValue == 0) {
        orpValue = newOrpValue;
    } else {
        orpValue = (9 * orpValue + newOrpValue) / 10;
    }
    
    if (millis() < streaming_enabled_until) {
        // Debug code to stream values, for filter tuning
        mqtt.publish("openbopi/pHstream", String(newPhValue, 2));
        mqtt.publish("openbopi/ORPstream", String(newOrpValue, 0));
    }
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

void mqtt_set_injection_time_cb(const char *payload)
{
    int time = atoi(payload);
    if (time > 0 && time <= 30) {  // Limit to 30 seconds max
        chlorine_injection_time = time;
        next_publish_at = millis();
    }
}

void mqtt_set_injection_interval_cb(const char *payload)
{
    int interval_seconds = atoi(payload);
    if (interval_seconds >= 60 && interval_seconds <= 7200) {  // Between 1 minute and 2 hours in seconds
        min_injection_interval = interval_seconds * 1000;
        next_publish_at = millis();
    }
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
    if (!orp_regulation_enabled) {
        if (pump_running[1]) {
            stop_pump(1);
        }
        return;
    }
   /* broken pH sensor so can't use this.. 
    if (phValue > 8.5) {
        mqtt.publish("openbopi/status", "pH too high for chlorine injection: suspected problem");
        toggle_ORP_regulation(0);
        stop_pump(1);
        return;
    }*/

    if (orpValue < 350) {
        mqtt.publish("openbopi/status", "ORP abnormally low");
        toggle_ORP_regulation(0);
        stop_pump(1);
        return;
    }
    
    unsigned long now = millis();
    if (orpValue < orp_target && 
        !pump_running[1] &&
        (now - last_chlorine_injection) > min_injection_interval) {
       
        last_chlorine_injection = now;
        start_pump(1, chlorine_injection_time);
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
        html.replace("%INJECTION_TIME%", String(chlorine_injection_time));
        html.replace("%INJECTION_INTERVAL%", String(min_injection_interval / 1000));
        html.replace("%FAST_PUBLISH%", millis() < fast_publish_until ? "ON" : "OFF");
        html.replace("%STREAMING%", millis() < streaming_enabled_until ? "ON" : "OFF");
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
            float requested_value = request->getParam("value")->value().toFloat();
            if (requested_value >= 300 && requested_value <= 800) {  // Safe ORP range
                orp_target = requested_value;
                next_publish_at = millis();  // Trigger immediate publish
            } else {
                request->send(400, "text/plain", "Invalid ORP target value. Must be between 300 and 800.");
                return;
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/injection_time", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            int time = request->getParam("value")->value().toInt();
            if (time > 0 && time <= 30) {  // Limit to 30 seconds max
                chlorine_injection_time = time;
                next_publish_at = millis();  // Trigger immediate publish
            } else {
                request->send(400, "text/plain", "Invalid injection time. Must be between 1 and 30 seconds.");
                return;
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/injection_interval", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            int interval_seconds = request->getParam("value")->value().toInt();
            if (interval_seconds >= 60 && interval_seconds <= 7200) {  // Between 1 minute and 2 hours in seconds
                min_injection_interval = interval_seconds * 1000;
                next_publish_at = millis();  // Trigger immediate publish
            } else {
                request->send(400, "text/plain", "Invalid injection interval. Must be between 60 and 7200 seconds.");
                return;
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/toggle_regulation", HTTP_GET, [](AsyncWebServerRequest *request) {
        toggle_ORP_regulation(!orp_regulation_enabled);
        request->send(200, "text/plain", orp_regulation_enabled ? "ON" : "OFF");
    });

    server.on("/fast_publish", HTTP_GET, [](AsyncWebServerRequest *request) {
        fast_publish_until = millis() + 60 * 60 * 1000; // 60 minutes
        next_publish_at = millis();
        request->send(200, "text/plain", "Fast publishing enabled");
    });

    server.on("/streaming", HTTP_GET, [](AsyncWebServerRequest *request) {
        streaming_enabled_until = millis() + 10 * 60 * 1000; // 10 minutes
        next_publish_at = millis();
        request->send(200, "text/plain", "Streaming enabled");
    });

    server.begin();
}

void setupMDNS() {
    if (!MDNS.begin("openbopi")) {
        Serial.println("Error setting up MDNS responder!");
        return;
    }
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started at openbopi.local");
}

void setup() {
    Serial.begin(115200);
    
    pinMode(pump_pins[0], OUTPUT);
    pinMode(pump_pins[1], OUTPUT);
    digitalWrite(pump_pins[0], LOW);
    digitalWrite(pump_pins[1], LOW);
   
    dht.begin();
    setupMDNS();
    setupWiFi();
    setupOTA();
    setupADC();
    setupTempSensors();
   
    mqtt.subscribe("openbopi/show_status", &mqtt_show_status_cb);
    mqtt.subscribe("openbopi/force_pump_10sec", &mqtt_force_pump_cb);
    mqtt.subscribe("openbopi/set_orp_target", &mqtt_set_orp_target_cb);
    mqtt.subscribe("openbopi/enable_orp_regulation", &mqtt_enable_orp_regulation_cb);
    mqtt.subscribe("openbopi/set_injection_time", &mqtt_set_injection_time_cb);
    mqtt.subscribe("openbopi/set_injection_interval", &mqtt_set_injection_interval_cb);

    mqtt.begin();
    mqtt.loop();
    setupWebServer();
    Serial.println("Openbopi hello");
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    
    uint32_t now = millis();
   
    read_pH_ORP();
    

    if (now > next_read_data_at) {
        readTemperatures();
        readDHT11();
        next_read_data_at = millis() + 5000;
    }

    if (now > next_publish_at) {
#define FAST_PUBLISH_INTERVAL 10 
#define REGULATION_PUBLISH_INTERVAL 1 * 60
#define NORMAL_PUBLISH_INTERVAL  10 * 60
        unsigned long publish_interval;
        if (now < fast_publish_until) {
            publish_interval = FAST_PUBLISH_INTERVAL;
        } else if (orp_regulation_enabled) {
            publish_interval = REGULATION_PUBLISH_INTERVAL;
        } else {
            publish_interval = NORMAL_PUBLISH_INTERVAL;
        }

        publishStatus();
        next_publish_at = now + publish_interval * 1000;
    }
    
    checkORPRegulation();
    checkPumps();

    const unsigned long REBOOT_AT_MILLIS = 4200000000UL;  // Reboot at ~48.6 days, safely before overflow at 49.7 days
    if (millis() > REBOOT_AT_MILLIS) {
        Serial.println("Approaching millis() overflow, rebooting...");
        mqtt.publish("openbopi/status", "Rebooting due to millis() overflow prevention");
        delay(100);  // Allow time for MQTT message to be sent
        ESP.restart();
    }

}

