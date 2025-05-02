#include <ESP8266WiFi.h>
#include <PicoMQTT.h>
#include <ESP8266WebServer.h>
#include "wifi_params.h"
#include "mqtt_login.h"
#include <ArduinoOTA.h>

#define SSR_PIN D8

ESP8266WebServer server(80);
PicoMQTT::Client mqtt(MQTT_IP, 1883, "triac_heat", MQTT_USER, MQTT_PASS);

// Control parameters
const unsigned long PERIOD_US = 250 * 1000;
const long DUTY_POINTS_TOTAL = PERIOD_US / 10000; // There are 100 half waves per second, one per 10ms. How many half-waves are there per period? This tells us our operating range in points (not watts) and granularity

// Temperature control parameters
const unsigned long TEMPERATURE_VALID_FOR_MS = 60 * 60 * 1000; // 1 hour timeout for temperature readings
const unsigned long ROOM_HOT_DEVICE_OFF_DURATION_MS = 60 * 60 * 1000; // 1 hour duration for temperature-based shutdown

// Temperature tracking variables
float room_temperature = 0;
float ext_temperature = 0;
unsigned long last_room_temp_update = 0;
unsigned long last_ext_temp_update = 0;
unsigned long device_off_until = 0; // When device is forced off due to temperature

// This heater has 3 maximum power settings, 600, 900 and 1500W. Determine
// which mode we are running in based on live consumption feedback. 
float radiator_max_power = 600;
float radiator_calculated_max_power = 600;

int duty_points; // How many half-waves to let through
uint32_t last_duty_change;

unsigned long next_publish_at = 0;
bool ssrState;
bool linky_sees_injection;

unsigned long forced_mode_until = 0;

bool radiator_security_thermostat_off = false;

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

float watts_to_duty_points(float w)
{
    // Duty points are number of half waves per period. Convert a watt figure
    // into that. Use float return so that the caller can floorf/ceilf based on
    // requirements.
   
    if (w > radiator_max_power) {
        return DUTY_POINTS_TOTAL;
    } else if (w <= 0) {
        return 0;
    }

    return (w / radiator_max_power) * DUTY_POINTS_TOTAL;
}

// Set the duty points, ensuring that it is within the valid range
// Returns true if the duty points were changed
bool set_duty_points(int new_duty_points)
{
    if (millis() < device_off_until) {
        // Temperature-based shutdown is active, ignore duty point changes
        new_duty_points = 0;
    }

    if (new_duty_points < 0) {
        new_duty_points = 0;
    } else if (new_duty_points > DUTY_POINTS_TOTAL) {
        new_duty_points = DUTY_POINTS_TOTAL;
    }

    if (duty_points != new_duty_points) {
        duty_points = new_duty_points;
        last_duty_change = millis();
        next_publish_at = millis();
        return true;
    }

    return false;
}

void PAPP_cb(const char *topic, const char *payload) 
{
    (void) topic;
    // Notification from Linky of current apparent power. 0 means we are
    // injecting (these smart people are purposefully hiding how much we are
    // injecting, no doubt in order to make it easy for us to consume our
    // surplus

    if (millis() < forced_mode_until) {
        // Forced mode active, ignore
        return;
    }

    int PAPP = atoi(payload);
    if (PAPP == 0) {
        linky_sees_injection = true;
    } else {
        linky_sees_injection = false;
    }

    if (!linky_sees_injection) {
        // We have stopped injecting: reduce consumed power by half or 50W, whichever is greater
        int reduce_50W = ceilf(watts_to_duty_points(50));
        int reduce_half = (duty_points + 1)/ 2;
        int reduce = reduce_50W > reduce_half ? reduce_50W : reduce_half;
        if (set_duty_points(duty_points - reduce)) {
            mqtt.publish("triac_heat/log", "Linky sees injection, reduced consumption");
        }
    } else {
        // We have started to inject. Use netpower_cb to increase power consumption.
    }
}

void PVprod_cb(const char *topic, const char *payload) 
{
    (void) topic;
    // Notification from inverter of amount of solar energy being produced.
    // This sets a hard cap on how much to consume.

    if (millis() < forced_mode_until) {
        // Forced mode is active, ignore
        return;
    }

    int PVprod = atoi(payload);
    int max_duty = floorf(watts_to_duty_points(PVprod));

    if (duty_points > max_duty) {
        if (set_duty_points(max_duty)) {
            mqtt.publish("triac_heat/log", "Solar production lower than current consumption, capped consumption");
        }
    }
}

void netpower_cb(const char *topic, const char *payload) 
{
    (void) topic;
    static uint32_t last_called;

    if (millis() < forced_mode_until) {
        // Forced mode is active, ignore consumption reports
        return;
    }

    if (millis() - last_called < 10*1000) {
        // There are redundant calls because Z2M sucks and reports the same
        // thing many times (each Zigbee message from the device triggers a
        // full report, so on MQTT the power consumption is reported 10
        // times). Filter out redundant calls and allow at most one every 10 seconds.
        // 
        return;
    }

    last_called = millis();

    // The main regulation objective is to keep this value close to zero.
    int netpower = atoi(payload);

    if (netpower > 0) {
        // We are consuming energy. The PAPP_cb, which gets injection information from the Linky, is the source of truth since we're trying to make sure that the Linky agrees we are not injecting. Let that callback handle things.
        return;
    }

    if (-netpower < 50) {
        // Do not increase consumption if we're injecting less than 50W.
        return;
    }

    if (millis() - last_duty_change < 15 * 1000) {
        // Do not increase consumption if the last duty cycle change was less than 15 seconds ago
        return;
    }

    if (radiator_security_thermostat_off) {
        // Do not increase consumption if the radiator is off despite a non-zero duty cycle.
        return;
    }

    // How much to increase the power consumption of the heater? 
    int increase = floorf(watts_to_duty_points(-netpower)) - 1;

    // Do not increase by more than 200W increments
    if (increase > watts_to_duty_points(200)) {
        increase = watts_to_duty_points(200);
    }

    if (increase > 0) {
        // set_duty_points may do nothing if already at max power
        if (set_duty_points(duty_points + increase)) {
            mqtt.publish("triac_heat/log", String("Injecting ") + String(-netpower) + String("W, increasing consumption"));
        }
    }
}

void myconsumption_cb(const char *topic, const char *payload) 
{
    (void) topic;

    // Notification from smart plug of current power consumption of the heater.
    if (millis() - last_duty_change < 15000) {
        // If we get a consumption report less than 15 seconds after changing duty cycle, ignore it, it is going to be outdated/incorrect.
        return;
    }

    // This is a sanity check from theoretical calculation of duty_points/DUTY_POINTS_TOTAL * 600W (depends on the setting of the heater).
    float myconsumption = atof(payload);
    if (myconsumption == 0.0 && duty_points > 0) {
        // The security thermostat on the radiator is off: this is unavoidable and expected behavior. Very unwelcome because the long term effective max duty cycle of this radiator turns out to be around 50%, which leads to unconsumable injection :(
        // If radiator is off despite a non-zero duty cycle, then keep track of this information so that we do not increase the duty cycle.
        radiator_security_thermostat_off = true;
        mqtt.publish("triac_heat/log", "Radiator security thermostat appears off (0 consumption with non-zero duty cycle).");
        return;
    }
    radiator_security_thermostat_off = false;

    int expected_dutypoints = watts_to_duty_points(myconsumption);

    if (abs(expected_dutypoints - duty_points) > 1) {
        mqtt.publish("triac_heat/log", String("Heater consumption ") + String(myconsumption) + String(" does not match expected consumption ") + String(radiator_max_power * duty_points / DUTY_POINTS_TOTAL));
    }

    if (duty_points == 0) {
        return;
    }

    // Exponential moving average of the heater's consumption. This is used to estimate its power setting.
    float old_max_power = radiator_max_power;
    radiator_calculated_max_power *= 3;
    radiator_calculated_max_power += myconsumption * DUTY_POINTS_TOTAL / duty_points;
    radiator_calculated_max_power /= 4;
    if (fabsf(radiator_calculated_max_power - 600) < 60) {
        radiator_max_power = 600;
    } else if (fabsf(radiator_calculated_max_power - 950) < 90) {
        radiator_max_power = 950; // the datasheet says 900W but I'm measuring closer to 950. temperature coefficient effects?
    } else if (fabsf(radiator_calculated_max_power - 1500) < 150) {
        radiator_max_power = 1500;
    } else {
        mqtt.publish("triac_heat/log", String("Heater max power calculated to be ") + String(radiator_calculated_max_power) + String(" does not match any allowed value (600, 950, 1500) "));
    }

    if (radiator_max_power != old_max_power) {
        mqtt.publish("triac_heat/log", String("Heater max power detected to be ") + String(radiator_max_power) + String("W, calculated to be ") + String(radiator_calculated_max_power) + String("W"));
    }

}

void roomtemperature_cb(const char *topic, const char * payload)
{
  (void) topic;
  float temperature = atof(payload);
  if (temperature < 5 || temperature > 35) {
    // Ignore invalid temperature
    return;
  }

  // Update room temperature and timestamp
  room_temperature = temperature;
  last_room_temp_update = millis();

  // Check temperature conditions
  check_temperature_shutdown_conditions();
}

void exttemperature_cb(const char *topic, const char *payload)
{
  (void) topic;
  float temperature = atof(payload);
  if (temperature < -20 || temperature > 50) {
    // Ignore invalid temperature
    return;
  }

  // Update external temperature and timestamp
  ext_temperature = temperature;
  last_ext_temp_update = millis();

  // Check temperature conditions
  check_temperature_shutdown_conditions();
}

void check_temperature_shutdown_conditions() {
  bool room_temp_valid = (millis() - last_room_temp_update) < TEMPERATURE_VALID_FOR_MS;
  bool ext_temp_valid = (millis() - last_ext_temp_update) < TEMPERATURE_VALID_FOR_MS;

  bool room_temp_wants_disable = false;
  bool ext_temp_wants_disable = false;
  bool do_disable = false;
  if (ext_temp_valid && ext_temperature >= 19) {
    // Exterior temperature of 19°C -> do not heat inside, no matter what the room temperature is
    do_disable = true;
  }

  if (room_temp_valid && room_temperature >= 21) {
    // Room temperature reached 21°C -> disable heating if exterior temperature
    // is below 17°C or if exterior temperature is not available
    if (!ext_temp_valid || ext_temperature > 17) {
      do_disable = true;
    }
  }

  if (do_disable) {
    device_off_until = millis() + ROOM_HOT_DEVICE_OFF_DURATION_MS;
    set_duty_points(0);
    mqtt.publish("triac_heat/log", "Room or exterior temp high enough: turning off for 1 hour");
  }
}

void setupMQTT() 
{
  mqtt.subscribe("triac_heat/setduty", [](const char* topic, const char* payload) {
    (void) topic;
    int newDuty = atoi(payload);
    set_duty_points(newDuty);
  });

  mqtt.subscribe("edf/PAPP", PAPP_cb);
  mqtt.subscribe("solar/ac/power", PVprod_cb);
  mqtt.subscribe("zigbee2mqtt/main_panel_powermonitor/power_ab", netpower_cb);
  mqtt.subscribe("zigbee2mqtt/smartplug_lidl/power", myconsumption_cb);
  mqtt.subscribe("zigbee2mqtt/bedroom_thermometer/temperature", roomtemperature_cb);
  mqtt.subscribe("exterior_thermometer/temperature", exttemperature_cb);

  mqtt.begin();
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    String html = "<html><head><title>Heater triac control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "</head><body>";
    html += "<h1>SSR Control</h1>";
    
    if (millis() < forced_mode_until) {
      unsigned long remaining = (forced_mode_until - millis()) / 60000;
      html += "<p style='color: red'>Forced mode active for " + String(remaining) + " more minutes</p>";
    }
    
    if (millis() < device_off_until) {
      unsigned long remaining = (device_off_until - millis()) / 60000;
      html += "<p style='color: blue'>Temperature-based shutdown active for " + String(remaining) + " more minutes</p>";
    }
    
    html += "<p>Current duty points: " + String(duty_points) + "/" + String(DUTY_POINTS_TOTAL) + "</p>";
    html += "<p>Detected radiator max power: " + String(radiator_max_power) + "</p><p>Current power draw: " + String(duty_points * radiator_max_power / DUTY_POINTS_TOTAL) + "W</p>";
    
    // Temperature information
    bool room_temp_valid = (millis() - last_room_temp_update) < TEMPERATURE_VALID_FOR_MS;
    bool ext_temp_valid = (millis() - last_ext_temp_update) < TEMPERATURE_VALID_FOR_MS;
    
    html += "<p>Room temperature: ";
    if (room_temp_valid) {
      html += String(room_temperature) + "°C";
    } else {
      html += "No recent data";
    }
    html += "</p>";
    
    html += "<p>External temperature: ";
    if (ext_temp_valid) {
      html += String(ext_temperature) + "°C";
    } else {
      html += "No recent data";
    }
    html += "</p>";
    
    // Single form for both duty setting and force mode
    html += "<form method='POST' action='/setduty'>";
    html += "<input type='number' step='1' min='0' max='" + String(DUTY_POINTS_TOTAL) + "' name='duty'>";
    html += "<input type='submit' value='Set'>";
    html += "<input type='submit' name='force' value='Force for 20 minutes'>";
    html += "</form></body></html>";
    
    server.send(200, "text/html", html);
  });
  
  server.on("/setduty", HTTP_POST, []() {
    if (server.hasArg("duty")) {
        int newDuty = server.arg("duty").toInt();
        set_duty_points(newDuty);
        
        // If force button was pressed, also set the timer
        if (server.hasArg("force")) {
            forced_mode_until = millis() + (20 * 60 * 1000);
            mqtt.publish("triac_heat/log", "Forced mode active for 20 minutes");
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
    if (duty_points == 0) {
        digitalWrite(SSR_PIN, LOW);
        ssrState = false;
    } else if (duty_points == DUTY_POINTS_TOTAL) {
        digitalWrite(SSR_PIN, HIGH);
        ssrState = true;
    } else {
        ssrState = !ssrState;
        digitalWrite(SSR_PIN, ssrState);
    }
    
    // Calculate next interval based on duty cycle
    uint32_t interval;
    if (ssrState) {
        interval = TIMER_TICKS((float)duty_points * PERIOD_US / DUTY_POINTS_TOTAL);
    } else {
        interval = TIMER_TICKS((float)(DUTY_POINTS_TOTAL - duty_points) * PERIOD_US / DUTY_POINTS_TOTAL);
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
    uint32_t initial_interval = TIMER_TICKS(PERIOD_US);
    timer1_write(initial_interval);
}

void publishStatus() {
    mqtt.publish("triac_heat/duty_points", String(duty_points));
    
    // Publish temperature control status
    if (millis() < device_off_until) {
        unsigned long remaining_mins = (device_off_until - millis()) / 60000;
        mqtt.publish("triac_heat/temp_shutdown_remaining", String(remaining_mins));
    } else {
        mqtt.publish("triac_heat/temp_shutdown_remaining", "0");
    }
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();
    server.handleClient();
   
    // Regular status updates
    if (millis() > next_publish_at) {
        publishStatus();
        next_publish_at = millis() + 15 * 60000;
    }

    // If uptime exceeds 45 days, reboot to prevent overflows
    if (millis() > 45 * 24 * 60 * 60 * 1000UL) {
        mqtt.publish("triac_heat/log", "Rebooting to prevent overflow");
        // Perform any cleanup needed
        WiFi.disconnect();
        delay(100);
        ESP.restart();
    }
}

