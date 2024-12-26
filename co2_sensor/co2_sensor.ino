#include <ESP8266WiFi.h>
#include <PicoMQTT.h>
#include <ModbusMaster.h>
#include <ESP8266WebServer.h>
#include "wifi_params.h"
#include "mqtt_login.h"
#include <SoftwareSerial.h>
#include <ArduinoOTA.h>

PicoMQTT::Client mqtt(MQTT_IP);
ESP8266WebServer server(80);

// Initialize software serial for S8 communication
SoftwareSerial s8Serial(D1, D2); // RX, TX

ModbusMaster s8modbus;

int last_co2_value;

void setupOTA() {
    // Port defaults to 8266
    ArduinoOTA.setPort(8266);
    
    // Set hostname for easy identification
    ArduinoOTA.setHostname("co2sensor");
    
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
        s8Serial.end();
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
    mqtt.username = MQTT_USER;
    mqtt.password = MQTT_PASS;
    
    mqtt.begin();
}

void setupWebServer()
{
    server.on("/", HTTP_GET, []() {
            server.send(200, "text/plain", String(last_co2_value)); });
}

void setup() {
  // Start hardware serial for debugging
  Serial.begin(115200);
  
  // Start software serial for S8 sensor
  s8Serial.begin(9600);
  s8modbus.begin(0xFE, s8Serial);
  
  Serial.println("SenseAir S8 CO2 Sensor Test");
    
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

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    server.handleClient();

    // https://rmtplusstoragesenseair.blob.core.windows.net/docs/Dev/publicerat/TDE2067.pdf
      // Read CO2 value using Modbus function 0x04 (Read Input Registers)
    uint8_t result = s8modbus.readInputRegisters(0x0003, 1);
    
    if (result == s8modbus.ku8MBSuccess) {
        int co2 = s8modbus.getResponseBuffer(0);
        Serial.print("CO2 concentration: ");
        Serial.print(co2);
        Serial.println(" ppm");
        mqtt.publish("co2sensor/ppm", String(co2));
        last_co2_value = co2;
    } else {
        Serial.print("Error reading CO2: ");
        Serial.println(result);
    }
        
  delay(10000);
}
