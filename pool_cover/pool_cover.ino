#include <ESP8266WiFi.h>
#include <PicoMQTT.h>
#include <ESP8266WebServer.h>
#include "wifi_params.h"
#include "mqtt_login.h"
#include <ArduinoOTA.h>

#define UP_PIN D2
#define DOWN_PIN D1

ESP8266WebServer server(80);
PicoMQTT::Client mqtt(MQTT_IP, 1883, "pool_cover", MQTT_USER, MQTT_PASS);

// Pool cover control parameters
const unsigned long DEFAULT_MOVEMENT_DURATION_MS = 60 * 1000; // 1 minute default movement time
const unsigned long SAFETY_TIMEOUT_MS = 5 * 60 * 1000; // 5 minutes maximum safety timeout

// Pool cover state
enum CoverState {
    STOPPED,
    GOING_UP,
    GOING_DOWN
};

CoverState current_state = STOPPED;
unsigned long movement_start_time = 0;
unsigned long movement_duration = DEFAULT_MOVEMENT_DURATION_MS;
bool safety_timeout_active = false;

unsigned long next_publish_at = 0;

void setupOTA() {
    // Port defaults to 8266
    ArduinoOTA.setPort(8266);
    
    // Set hostname for easy identification
    ArduinoOTA.setHostname("PoolCover");
    
    // Set password for OTA updates
    ArduinoOTA.setPassword("pool123");  

    ArduinoOTA.onStart([]() {
        // Stop all movement before OTA begins
        stopCover();
        mqtt.disconnect();

        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("Starting OTA update (" + type + ")");
        
        // Stop MQTT and other communications during update
        mqtt.disconnect();
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

void setPins(bool up, bool down) {
    digitalWrite(UP_PIN, up ? HIGH : LOW);
    digitalWrite(DOWN_PIN, down ? HIGH : LOW);
}

void stopCover() {
    setPins(false, false);
    current_state = STOPPED;
    movement_start_time = 0;
    safety_timeout_active = false;
    mqtt.publish("pool_cover/log", "Pool cover stopped");
}

void startUp() {
    if (current_state == GOING_UP) {
        // Already going up, restart timer
        movement_start_time = millis();
        mqtt.publish("pool_cover/log", "Pool cover up movement restarted");
        return;
    }
    
    stopCover(); // Stop any current movement
    setPins(true, false); // 1 0 = go up (corrected)
    current_state = GOING_UP;
    movement_start_time = millis();
    mqtt.publish("pool_cover/log", "Pool cover going up");
}

void startDown() {
    if (current_state == GOING_DOWN) {
        // Already going down, restart timer
        movement_start_time = millis();
        mqtt.publish("pool_cover/log", "Pool cover down movement restarted");
        return;
    }
    
    stopCover(); // Stop any current movement
    setPins(false, true); // 0 1 = go down (corrected)
    current_state = GOING_DOWN;
    movement_start_time = millis();
    mqtt.publish("pool_cover/log", "Pool cover going down");
}

void checkMovementTimeout() {
    if (current_state == STOPPED) {
        return;
    }
    
    unsigned long elapsed = millis() - movement_start_time;
    
    // Check if movement duration has elapsed
    if (elapsed >= movement_duration) {
        stopCover();
        mqtt.publish("pool_cover/log", "Movement timeout reached, stopping");
        return;
    }
    
    // Check safety timeout (should never happen if movement_duration is set correctly)
    if (elapsed >= SAFETY_TIMEOUT_MS) {
        stopCover();
        mqtt.publish("pool_cover/log", "SAFETY TIMEOUT: Emergency stop activated");
        return;
    }
}

void setupMQTT() 
{
    mqtt.subscribe("pool_cover/up", [](const char* topic, const char* payload) {
        (void) topic;
        (void) payload;
        startUp();
    });

    mqtt.subscribe("pool_cover/down", [](const char* topic, const char* payload) {
        (void) topic;
        (void) payload;
        startDown();
    });

    mqtt.subscribe("pool_cover/stop", [](const char* topic, const char* payload) {
        (void) topic;
        (void) payload;
        stopCover();
    });

    mqtt.subscribe("pool_cover/duration", [](const char* topic, const char* payload) {
        (void) topic;
        unsigned long new_duration = atol(payload);
        if (new_duration > 0 && new_duration <= SAFETY_TIMEOUT_MS) {
            movement_duration = new_duration;
            mqtt.publish("pool_cover/log", String("Movement duration set to ") + String(movement_duration) + String("ms"));
        } else {
            mqtt.publish("pool_cover/log", "Invalid duration value");
        }
    });

    mqtt.begin();
}

void setupWebServer() {
    server.on("/", HTTP_GET, []() {
        String html = "<html><head><title>Pool Cover Control</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; margin: 20px; }";
        html += ".button { padding: 15px 30px; margin: 10px; font-size: 18px; border: none; border-radius: 5px; cursor: pointer; }";
        html += ".up { background-color: #4CAF50; color: white; }";
        html += ".down { background-color: #f44336; color: white; }";
        html += ".stop { background-color: #ff9800; color: white; }";
        html += ".status { padding: 10px; margin: 10px 0; border-radius: 5px; }";
        html += ".stopped { background-color: #e0e0e0; }";
        html += ".moving { background-color: #fff3cd; }";
        html += "</style>";
        html += "</head><body>";
        html += "<h1>Pool Cover Control</h1>";
        
        // Status display
        html += "<div class='status ";
        if (current_state == STOPPED) {
            html += "stopped'>Status: STOPPED";
        } else if (current_state == GOING_UP) {
            html += "moving'>Status: GOING UP";
            unsigned long elapsed = (millis() - movement_start_time) / 1000;
            unsigned long remaining = (movement_duration - (millis() - movement_start_time)) / 1000;
            html += " (" + String(elapsed) + "s elapsed, " + String(remaining) + "s remaining)";
        } else if (current_state == GOING_DOWN) {
            html += "moving'>Status: GOING DOWN";
            unsigned long elapsed = (millis() - movement_start_time) / 1000;
            unsigned long remaining = (movement_duration - (millis() - movement_start_time)) / 1000;
            html += " (" + String(elapsed) + "s elapsed, " + String(remaining) + "s remaining)";
        }
        html += "</div>";
        
        html += "<p>Current movement duration: " + String(movement_duration / 1000) + " seconds</p>";
        
        // Control buttons
        html += "<form method='POST' action='/control' style='display: inline;'>";
        html += "<button type='submit' name='action' value='up' class='button up'>UP</button>";
        html += "<button type='submit' name='action' value='down' class='button down'>DOWN</button>";
        html += "<button type='submit' name='action' value='stop' class='button stop'>STOP</button>";
        html += "</form>";
        
        // Duration setting
        html += "<h3>Set Movement Duration</h3>";
        html += "<form method='POST' action='/duration'>";
        html += "<input type='number' name='duration' min='10' max='300' value='" + String(movement_duration / 1000) + "' step='1'> seconds";
        html += "<input type='submit' value='Set Duration'>";
        html += "</form>";
        
        html += "</body></html>";
        
        server.send(200, "text/html", html);
    });
    
    server.on("/control", HTTP_POST, []() {
        if (server.hasArg("action")) {
            String action = server.arg("action");
            if (action == "up") {
                startUp();
            } else if (action == "down") {
                startDown();
            } else if (action == "stop") {
                stopCover();
            }
        }
        server.sendHeader("Location", "/");
        server.send(302);
    });
    
    // Handle GET requests to /control (redirect to home)
    server.on("/control", HTTP_GET, []() {
        server.sendHeader("Location", "/");
        server.send(302);
    });
    
    server.on("/duration", HTTP_POST, []() {
        if (server.hasArg("duration")) {
            unsigned long new_duration = server.arg("duration").toInt() * 1000; // Convert to milliseconds
            if (new_duration > 0 && new_duration <= SAFETY_TIMEOUT_MS) {
                movement_duration = new_duration;
                mqtt.publish("pool_cover/log", String("Movement duration set to ") + String(movement_duration / 1000) + String(" seconds"));
            }
        }
        server.sendHeader("Location", "/");
        server.send(302);
    });
    
    // Handle GET requests to /duration (redirect to home)
    server.on("/duration", HTTP_GET, []() {
        server.sendHeader("Location", "/");
        server.send(302);
    });
    
    server.begin();
}

void setup() {
    Serial.begin(115200);

    pinMode(UP_PIN, OUTPUT);
    pinMode(DOWN_PIN, OUTPUT);
    setPins(false, false); // Ensure both pins are low at startup

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

    Serial.println("Pool cover control system ready");
}

void publishStatus() {
    String state_str;
    switch (current_state) {
        case STOPPED:
            state_str = "stopped";
            break;
        case GOING_UP:
            state_str = "going_up";
            break;
        case GOING_DOWN:
            state_str = "going_down";
            break;
    }
    
    mqtt.publish("pool_cover/state", state_str);
    
    if (current_state != STOPPED) {
        unsigned long elapsed = (millis() - movement_start_time) / 1000;
        unsigned long remaining = (movement_duration - (millis() - movement_start_time)) / 1000;
        mqtt.publish("pool_cover/elapsed", String(elapsed));
        mqtt.publish("pool_cover/remaining", String(remaining));
    } else {
        mqtt.publish("pool_cover/elapsed", "0");
        mqtt.publish("pool_cover/remaining", "0");
    }
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    server.handleClient();
    
    // Check for movement timeout
    checkMovementTimeout();
   
    // Regular status updates
    if (millis() > next_publish_at) {
        publishStatus();
        next_publish_at = millis() + 5 * 1000; // Publish every 5 seconds
    }

    // If uptime exceeds 45 days, reboot to prevent overflows
    if (millis() > 45 * 24 * 60 * 60 * 1000UL) {
        mqtt.publish("pool_cover/log", "Rebooting to prevent overflow");
        // Perform any cleanup needed
        stopCover();
        WiFi.disconnect();
        delay(100);
        ESP.restart();
    }
}

