#include <ESP8266WiFi.h>
#include <PicoMQTT.h>
#include <ESP8266WebServer.h>
#include "wifi_params.h"
#include "mqtt_login.h"
#include <ArduinoOTA.h>

#define SSR_PIN D8

ESP8266WebServer server(80);
PicoMQTT::Client mqtt(MQTT_IP);

// Control parameters
float dutyCycle = 0.0f;  // Range 0.0-1.0
const unsigned long PERIOD_US = 500 * 1000;
unsigned long lastCycleStart = 0;

unsigned long next_publish_at = 0;

bool ssrState;

void setupOTA() {
    // Port defaults to 8266
    ArduinoOTA.setPort(8266);
    
    // Set hostname for easy identification
    ArduinoOTA.setHostname("TriacRadiator");
    
    // Set password for OTA updates
    ArduinoOTA.setPassword("triac123");  

    ArduinoOTA.onStart([]() {
        // Disable timer interrupt before OTA begins
        timer1_detachInterrupt();
        timer1_disable();
        digitalWrite(SSR_PIN, LOW);
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


void setupMQTT() {
  mqtt.subscribe("ssr/setduty", [](const char* topic, const char* payload) {
    float newDuty = atof(payload);
    if (newDuty >= 0.0f && newDuty <= 1.0f) {
      dutyCycle = newDuty;
      mqtt.publish("ssr/status", String(dutyCycle).c_str());
    }
  });
  
  mqtt.begin();
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    String html = "<html><body>";
    html += "<h1>SSR Control</h1>";
    html += "<p>Current duty cycle: " + String(dutyCycle) + "</p>";
    html += "<form method='POST' action='/setduty'>";
    html += "<input type='number' step='0.01' min='0' max='1' name='duty'>";
    html += "<input type='submit' value='Set'>";
    html += "</form></body></html>";
    server.send(200, "text/html", html);
  });
  
  server.on("/setduty", HTTP_POST, []() {
    if (server.hasArg("duty")) {
      float newDuty = server.arg("duty").toFloat();
      if (newDuty >= 0.0f && newDuty <= 1.0f) {
        dutyCycle = newDuty;
        mqtt.publish("ssr/status", String(dutyCycle).c_str());
      }
    }
    server.sendHeader("Location", "/");
    server.send(302);
  });
  
  server.begin();
}

#define CPU_FREQ 80000000L
#define TIMER_PRESCALER 16
#define TIMER_TICKS(us) ((CPU_FREQ / TIMER_PRESCALER) * (us) / 1000000)
void ICACHE_RAM_ATTR onTimerISR() {
    // Toggle SSR state based on duty cycle
    if (dutyCycle <= 0.0f) {
        digitalWrite(SSR_PIN, LOW);
        ssrState = false;
    } else if (dutyCycle >= 1.0f) {
        digitalWrite(SSR_PIN, HIGH);
        ssrState = true;
    } else {
        ssrState = !ssrState;
        digitalWrite(SSR_PIN, ssrState);
    }
    
    // Calculate next interval based on duty cycle
    uint32_t interval;
    if (ssrState) {
        interval = TIMER_TICKS(dutyCycle * PERIOD_US);
    } else {
        interval = TIMER_TICKS((1.0f - dutyCycle) * PERIOD_US);
    }
    
    timer1_write(interval);
}

void setup() {
    Serial.begin(115200);

    pinMode(SSR_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);

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

    // Initialize timer with microsecond precision
    timer1_isr_init();
    timer1_attachInterrupt(onTimerISR);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);

    // Start the timer with initial interval
    uint32_t initial_interval = TIMER_TICKS(PERIOD_US * (1.0f - dutyCycle));
    timer1_write(initial_interval);
}

void publishStatus() {
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    server.handleClient();
   
    // Regular status updates
    if (millis() > next_publish_at) {
        publishStatus();
        next_publish_at = millis() + 3 * 60000;
    }
}

