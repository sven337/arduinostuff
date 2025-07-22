#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <user_interface.h>
#include "wifi_params.h"

#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include <PicoMQTT.h>
#include "mqtt_login.h"
#include "therm_names.h"

ESP8266WebServer websrv (80);

WiFiClient client;

PicoMQTT::Client mqtt(MQTT_IP, 1883, "boiler", MQTT_USER, MQTT_PASS);

#define ARRAY_SZ(A) sizeof(A)/sizeof(A[0])

#define makeTRV(TRV) TRV, 0, 0, 0, 0

struct TRV {
    const char *name;
    float target_temp;
    float current_temp;
    bool active;
    uint32_t last_seen;
} TRVs[] = { 
    { makeTRV(TRV_BED1) },
    { makeTRV(TRV_BED2) },
    { makeTRV(TRV_BED3) },
    { makeTRV(TRV_BATH) },
};

#define makeTherm(TH, Z2M) TH, 0, 0, Z2M
struct thermometer {
    const char *name;
    float current_temp;
    uint32_t last_seen;
    bool is_Z2M;
} thermometers[] = {
    { makeTherm("exterior_thermometer", 0) },
    { makeTherm("living_thermometer", 1) },
//    { makeTherm("bedroom_thermometer") },
    { makeTherm(THERM_BED1, 1) },
    { makeTherm(THERM_BED2, 1) },
    { makeTherm(THERM_BED3, 0) },
    { makeTherm("controlepoele", 0) },
}; 

const int chaudiere = D5;
const int pushbtn = 12;

unsigned long int forced_heating_until = 0;
unsigned long int send_next_ping_at = 15*60*1000;

bool pushbtn_pressed = false;

int boiler_val = 0;

enum MODE {
    AWAY = 0, // 7°
    COLD = 1, // target 17°
    HOT = 2, // target 20°
    FORCED = 3 // force heating for 20 minutes 
} current_mode;

enum MODE previous_mode;
enum MODE next_mode;
bool mode_change_queued;

const char *mode_names[] = {
    [ AWAY ] = "away",
    [ COLD ] = "cold",
    [ HOT ]  = "hot",
    [ FORCED ] = "forced",
};

// Target temperature of thermostat
float thermostat_target_temp; 

char logStr[1024];

// Stove status
int stove_status;
uint32_t stove_status_since;
// Hold off triggering the boiler to let the stove heat up the room
uint32_t holdoff_for_stove_until;

// Enqueue a mode change, to avoid mqtt receive callbacks calling mqtt.publish which crashes
void queue_change_mode(enum MODE mode)
{
    next_mode = mode;
    mode_change_queued = true;
}

void handle_change_mode()
{
    if (!mode_change_queued) 
        return;

    mode_change_queued = false;
   
    if (next_mode == FORCED) {
        if (current_mode != FORCED) {
            // Store previous mode to revert to it when done
            previous_mode = current_mode;
        }
        current_mode = FORCED;
    } else if (next_mode == current_mode) {
        return;
    }

    current_mode = next_mode;
    
    switch (current_mode) {
        case FORCED:
            set_boiler(1);

            // Force heating for 20 minutes
            forced_heating_until = millis() + 20 * 60 * 1000; 

            // Overflow: ignore
            if (forced_heating_until < millis()) {
                forced_heating_until = 1;
            }
            Serial.println("Forcing heating");
            break;
        case AWAY:
            forced_heating_until = 0;
            // Frost protection
            thermostat_target_temp = 7.0;
            set_boiler(0);
            break;
        case HOT:
            thermostat_target_temp = 20.5;
            forced_heating_until = 0;
            break;
        case COLD: 
            thermostat_target_temp = 17.0;
            forced_heating_until = 0;
            break;
    }
   
    // ... and report status right away
    send_next_ping_at = 0; 
}

/* Web interface */
static void handleHot()
{
    queue_change_mode(HOT);
    sprintf(logStr, "Set HOT mode from http\n");
    websrv.sendHeader("Location", "/", true);
    websrv.send(302, "text/plain", "");
}

static void handleCold()
{
    queue_change_mode(COLD);
    sprintf(logStr, "Set COLD mode from http\n");
    websrv.sendHeader("Location", "/", true);
    websrv.send(302, "text/plain", "");
}

static void handleForceOn()
{
    queue_change_mode(FORCED);
    sprintf(logStr, "Set FORCED mode from http\n");
    websrv.sendHeader("Location", "/", true);
    websrv.send(302, "text/plain", "");
}

char tempBuf[4096];
char lastLogStr[1024];

static void handleRoot() {
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

    websrv.setContentLength(CONTENT_LENGTH_UNKNOWN);
    websrv.send(200, "text/html", "");
	snprintf(tempBuf, 4096,
"<html><meta http-equiv=\"refresh\" content=\"30\" />\
<head><title>Boiler control</title></head><body>\
    <h1>Boiler control</h1><p>Built on %s at %s</p><p>Uptime: %02d:%02d:%02d = %lu ms</p>\
	<p>Mode is %s, boiler heat request is %d, %sforced until %d, %d minutes from now</p> \
    <p>Target temp %.1f, %sholding off for boiler until %d, %d minutes from now</p> \
    <p><a href=\"/hot\"><button class=\"button\">HOT</button></a>\
    <a href=\"/cold\"><button class=\"button\">COLD</button></a>\
    <a href=\"/forceon\"><button class=\"button\">FORCE 20 min</button></a></p>\
    <table><thead><tr><th>Valve</th><th>Active</th><th>Current</th><th>Target</th><th>Last seen</th></tr></thead><tbody>",
		__DATE__, __TIME__, hr, min % 60, sec % 60, (uint32_t)millis(),
        mode_names[current_mode],
		boiler_val, (current_mode == FORCED ? "" : "not "), (uint32_t)forced_heating_until, (int)((forced_heating_until - millis()) / 60 / 1000),
        thermostat_target_temp, ((millis() > holdoff_for_stove_until) ? "not " : ""), holdoff_for_stove_until, (int)((holdoff_for_stove_until - millis()) / 60 / 1000) 
	);
    websrv.sendContent(tempBuf);


    tempBuf[0] = 0; 
    for (unsigned int i = 0; i < ARRAY_SZ(TRVs); i++) {
        sprintf(tempBuf+strlen(tempBuf), "<tr><td>%s</td><td>%d</td><td>%.1f C</td><td>%.1f C</td><td>%d min ago</td></tr>", TRVs[i].name, TRVs[i].active, TRVs[i].current_temp, TRVs[i].target_temp, (int)(millis() - TRVs[i].last_seen)/(60 * 1000));
    } 
   
    websrv.sendContent(tempBuf); 
    tempBuf[0] = 0; 
    websrv.sendContent("<table><thead><tr><th>Thermometer</th><th>Current</th><th>Last seen</th></tr></thead><tbody>");
    for (unsigned int i = 0; i < ARRAY_SZ(thermometers); i++) {
        sprintf(tempBuf+strlen(tempBuf), "<tr><td>%s</td><td>%.1f C</td><td>%d min ago</td></tr>", thermometers[i].name, thermometers[i].current_temp, (int)(millis() - thermometers[i].last_seen)/(60 * 1000));
    } 


    strcat(tempBuf, lastLogStr);
    strcat(tempBuf, "</body></html>");
	websrv.sendContent(tempBuf);
    websrv.client().stop();
}

/* Physical button */
static void IRAM_ATTR pushbtn_intr(void)
{
	if (!digitalRead(pushbtn)) {
		pushbtn_pressed = true;
	}
}

/* OTA */
void ota_onprogress(unsigned int sz, unsigned int total)
{
	Serial.print("OTA: "); Serial.print(sz); Serial.print("/"); Serial.print(total);
	Serial.print("="); Serial.print(100*sz/total); Serial.println("%%");
}

void ota_onerror(ota_error_t err)
{
	Serial.print("OTA ERROR:"); Serial.println((int)err);
}

/* MQTT */
struct TRV *find_TRV(const char *name)
{
    for (unsigned int i = 0; i < ARRAY_SZ(TRVs); i++) {
        if (!strcmp(TRVs[i].name, name)) 
            return &TRVs[i];
    }
    sprintf(logStr, "Cannot find TRV for %s:", name);
    return NULL;
}

void mqtt_change_mode_cb(const char *payload)
{
    for (uint16_t i = 0; i < ARRAY_SZ(mode_names); i++) {
        if (!strcmp(payload, mode_names[i])) {
            queue_change_mode((enum MODE)i);
        }
    }
}

void mqtt_change_target_temp_cb(const char *payload)
{
    float target = atof(payload);
    if (target > 0 && target < 30) {
        thermostat_target_temp = target;
        // Force a ping to be sent
        send_next_ping_at = 0;
    }
}

void mqtt_TRV_systemmode_cb(const char *topic, const char *payload)
{
    String TRV_name_s = mqtt.get_topic_element(topic, 1);
    const char *TRV_name = TRV_name_s.c_str();
    if (!TRV_name) {
        strcpy(logStr, "TRV name is NULL");
        return;
    }

    struct TRV *trv = find_TRV(TRV_name);
    if (!trv) {
        strcpy(logStr, "Cannot identify TRV name");
        return;
    }

    if (!strcmp(payload, "off")) {
        trv->active = false;
    } else if (!strcmp(payload, "heat")) {
        trv->active = true;
    } else {
        sprintf(logStr, "Cannot identify TRV state %s for %s", payload, topic);
    }

    trv->last_seen = millis();
}

void mqtt_TRV_temp_handle(const char *topic, const char *payload, uint32_t offset)
{
    String TRV_name_s = mqtt.get_topic_element(topic, 1);
    const char *TRV_name = TRV_name_s.c_str();

    struct TRV *trv = find_TRV(TRV_name);
    if (!trv) {
        strcpy(logStr, "Cannot identify TRV name");
        return;
    }

    float temp = atof(payload);
    if (temp < 0 || temp > 40) {
        sprintf(logStr, "Temperature seems bogus: %s", payload);
        return;
    }

    float *temp_value = (float *)((char *)trv + offset);
    *temp_value = temp;
//    sprintf(logStr, "Writing temperature %.1f at %p trv is %p, %s = %s\n", temp, temp_value, trv, topic, payload);
    
    trv->last_seen = millis();
}

void mqtt_TRV_targettemp_cb(const char *topic, const char *payload)
{
    mqtt_TRV_temp_handle(topic, payload, offsetof(struct TRV, target_temp));
}

void mqtt_TRV_currenttemp_cb(const char *topic, const char *payload)
{
    mqtt_TRV_temp_handle(topic, payload, offsetof(struct TRV, current_temp));
}

struct thermometer *find_thermometer(const char *name)
{
    for (unsigned int i = 0; i < ARRAY_SZ(thermometers); i++) {
        if (!strcmp(thermometers[i].name, name)) 
            return &thermometers[i];
    }
    sprintf(logStr, "Cannot find thermometer for %s:", name);
    return NULL;
}

void mqtt_thermometer_cb(const char *topic, const char *payload)
{
    int topic_offset = 0;
    if (strstr(topic, "zigbee2mqtt")) {
        topic_offset = 1;
    }
    String thermometer_name_s = mqtt.get_topic_element(topic, topic_offset);
    const char *therm_name = thermometer_name_s.c_str();

    struct thermometer *therm = find_thermometer(therm_name);
    if (!therm) {
        sprintf(logStr, "Cannot identify thermometer with name %s", therm_name);
        return;
    }

    float temp = atof(payload);
    if (strcmp(therm_name, "exterior_thermometer") && (temp < 5 || temp > 40)) {
        // sanity check (except for exterior thermometer)
        sprintf(logStr, "Temperature seems bogus: %s", payload);
        return;
    }

    therm->current_temp = temp;
    therm->last_seen = millis();
}

void mqtt_stovestatus_cb(const char *topic, const char *payload)
{
    int new_status = atoi(payload);
    if (new_status != stove_status) {
        stove_status = new_status;
        stove_status_since = millis();
    }
}

bool last_seen_too_old(uint32_t last_seen)
{
    // Boot 
    if (last_seen == 0) {
        return true;
    }

    // Overflow
    if (last_seen > millis()) {
        return true;
    }

    // Last update > 1h ago
    if ((millis() - last_seen) > 60 * 60 * 1000) {
        return true;
    }

    return false;
}

/* Do the TRVs require heating, that is, is their target temperature above the current temperature in the room? */
bool TRV_requires_heat(void)
{
    bool heat = false;

    struct {
        const char *TRV;
        const char *thermometer;
    } list[] = {
        { TRV_BED1, THERM_BED1 },
        { TRV_BED2, THERM_BED2 },
        { TRV_BED3, THERM_BED3 },
        { TRV_BATH,     NULL },
    };

    for (unsigned int i = 0; i < ARRAY_SZ(list); i++) {
        struct TRV *trv = find_TRV(list[i].TRV);

        if (!trv->active) {
            continue;
        }

        if (last_seen_too_old(trv->last_seen)) {
            sprintf(logStr, "TRV %s is too old, ignoring", trv->name);
        }

        // If the current temperature seen by the TRV is greater than the
        // target temp, then it is going to be closed anyway, so triggering the
        // boiler will have no effect.
        if (trv->current_temp > trv->target_temp) {
            continue;
        }

        float room_temp = trv->current_temp;

        // If a room thermometer is provided, average out TRV temperature with
        // that thermometer's
        if (list[i].thermometer) {
            struct thermometer *therm = find_thermometer(list[i].thermometer);
            if (last_seen_too_old(therm->last_seen)) {
                sprintf(logStr, "therm %s too old", therm->name);
            } else {
                room_temp += therm->current_temp;
                room_temp /= 2;
            }
        }


        if (room_temp <= trv->target_temp) {
            sprintf(logStr, "TRV %s requires heat", trv->name);
            heat = true;
        }
    }
        
    return heat;
}

/* The living room needs to be heated. Do we trigger the stove instead?
   Wood pellets are cheaper than natural gas so this takes absolute priority if
   the stove is in a state compatible with being triggered.
   */
bool heat_living_with_stove(void)
{
/*                '0' : 'Off',
            '1' : 'OffTimer',
            '2' : 'TestFire',
              '3' : 'Heatup',
              '4' : 'Fueling', <<- this one needs to be avoided see https://github.com/Domochip/WirelessPalaControl/issues/36
              '5' : 'IGNTEST',
              '6' : 'BURNING',
              '9' : 'COOLFLUID',
             '10' : 'FIRESTOP',
             '11' : 'CLEANFIRE',
             '12' : 'COOL',
            '239' : 'MFDOOR ALARM',
            '240' : 'FIRE ERROR',
            '241' : 'CHIMNEY ALARM',
            '243' : 'GRATE ERROR',
            '244' : 'NTC2 ALARM',
            '245' : 'NTC3 ALARM',
            '247' : 'DOOR ALARM',
            '248' : 'PRESS ALARM',
            '249' : 'NTC1 ALARM',
            '250' : 'TC1 ALARM',
            '252' : 'GAS ALARM',
            '253' : 'NOPELLET ALARM'} %}*/
    bool start_stove = false;
    switch (stove_status) {
        case 0:
        case 1:
        case 2:
        case 9:
            start_stove = true;
            break;
    }

    if (millis() < holdoff_for_stove_until) {
        // Holding off heating, stove is already starting
        strcat(logStr, "--> holding off for stove");
        return true;
    }

    if (start_stove) {
        mqtt.publish("controlepoele/cmd", "CMD+ON");
        strcat(logStr, "--> starting stove");
        holdoff_for_stove_until = millis() + 20 * 60 * 1000;
        return true;
    }

    return false;
}

/* Do temperatures in non-TRV rooms require heat? */
bool thermostat_requires_heat(void)
{
    static float last_living_temp = 0.0;
    float living_temp = 0;

    int cnt = 0;
    struct thermometer *living = find_thermometer("living_thermometer");
    struct thermometer *stove = find_thermometer("controlepoele");

    if (!last_seen_too_old(living->last_seen)) {
        living_temp += living->current_temp;
        cnt++;
    } else {
        sprintf(logStr, "living therm too old: %u now is %u delta %d", living->last_seen, (int)millis(), (int)millis() - living->last_seen);
    }

    if (!last_seen_too_old(stove->last_seen)) {
        living_temp += stove->current_temp;
        cnt++;
    } else {
        sprintf(logStr, "stove therm too old: %u now is %u delta %d", stove->last_seen, (int)millis(), (int)millis() - stove->last_seen);
    }

    if (cnt) {
        living_temp /= cnt;
    } else {
        living_temp = 0;
    } 

    // Publish the current computed temp
    if (living_temp != last_living_temp) {
        char buf[10];
        sprintf(buf, "%.1f", living_temp);
        mqtt.publish("boiler/current_living_temp", buf);
        last_living_temp = living_temp;
    }

    if (living_temp > 0 && living_temp < thermostat_target_temp) {
        // XXX add hysteresis
        sprintf(logStr, "Living avg temp %f target %f: requesting heat", living_temp, thermostat_target_temp);
        return !heat_living_with_stove();
    }

    // XXX take exterior temp into account
    return false;
}

void setup ( void ) {
	pinMode ( chaudiere, OUTPUT );
	pinMode(pushbtn, INPUT_PULLUP);
	Serial.begin ( 115200 );
	
	WiFi.mode(WIFI_STA);
	WiFi.begin ("agoctrl_EXT", password );
	Serial.println ("Boiler control starting, connecting to wifi.");

	// Wait for connection
	while ( WiFi.status() != WL_CONNECTED ) {
		delay ( 500 );
		Serial.print ( "." );
	}

	Serial.println ( "" );
	Serial.print ( "Cnnectd to " );
	Serial.println ( ssid );
	Serial.print ( "IP " );
	Serial.println ( WiFi.localIP() );

	websrv.on ( "/", handleRoot );
	websrv.on ( "/hot", handleHot);
	websrv.on ( "/forceon", handleForceOn );
	websrv.on ( "/cold", handleCold);

	websrv.begin();

    ArduinoOTA.onError(ota_onerror);
    ArduinoOTA.onProgress(ota_onprogress);
    ArduinoOTA.setHostname("boiler-control");
    ArduinoOTA.begin();
		  
	attachInterrupt(digitalPinToInterrupt(pushbtn), pushbtn_intr, FALLING);
    set_boiler(0);
    queue_change_mode(COLD);

    mqtt.subscribe("boiler/change_mode", &mqtt_change_mode_cb);
    mqtt.subscribe("boiler/change_target_temp", &mqtt_change_target_temp_cb);

    for (uint16_t v = 0; v < sizeof(TRVs)/sizeof(TRVs[0]); v++) {
        char buf[256] = "zigbee2mqtt/";
        char *name = &buf[0] + strlen(buf);
        strcpy(name, TRVs[v].name);

        char *topic = &buf[0] + strlen(buf);
        strcpy(topic, "/system_mode");
        mqtt.subscribe(buf, &mqtt_TRV_systemmode_cb);
        strcpy(topic, "/occupied_heating_setpoint");
        mqtt.subscribe(buf, &mqtt_TRV_targettemp_cb);
        strcpy(topic, "/local_temperature");
        mqtt.subscribe(buf, &mqtt_TRV_currenttemp_cb);
    }

    for (uint16_t t = 0; t < sizeof(thermometers)/sizeof(thermometers[0]); t++) {
        char buf[256];
        if (thermometers[t].is_Z2M) {
            strcpy(buf, "zigbee2mqtt/");
        } else {
            buf[0] = 0;
        }
        char *name = &buf[0] + strlen(buf);
        strcpy(name, thermometers[t].name);

        char *topic = &buf[0] + strlen(buf);
        strcpy(topic, "/temperature");
        mqtt.subscribe(buf, &mqtt_thermometer_cb);
    }    
    
    mqtt.will.topic = "boiler/available";
    mqtt.will.payload = "offline";
    mqtt.will.qos = 1;
    mqtt.will.retain = true;

    mqtt.subscribe("controlepoele/STATUS", &mqtt_stovestatus_cb);

    mqtt.begin();

    mqtt.loop();
    mqtt.publish("boiler/available", "online", 1, true);
    mqtt.publish("boiler/TAstatus", "0");
}

void mqtt_publish_TAstatus(bool val)
{
    char buf[2] = { (char)('0' + val), 0 };
    mqtt.publish("boiler/TAstatus", buf);
}

void mqtt_publish_target_temp()
{
    char buf[10];
    sprintf(buf, "%.1f", thermostat_target_temp);
    mqtt.publish("boiler/thermostat_target_temp", buf);
}

void set_boiler(bool val)
{
    digitalWrite(chaudiere, val);

    if (val != boiler_val) {
        mqtt_publish_TAstatus(val);
    }
    
    boiler_val = val;
}

void loop (void) {
    ArduinoOTA.handle();

    mqtt.loop();
	websrv.handleClient();

	if (pushbtn_pressed && !digitalRead(pushbtn)) {
		delay(5);
		if (!digitalRead(pushbtn)) {
            queue_change_mode(FORCED);
			pushbtn_pressed = false;
		}
	}

    if (current_mode == FORCED) {
        // End of forced heating
	    if (millis() > forced_heating_until) {
		    forced_heating_until = 0;
            queue_change_mode(previous_mode);
        }
    } else {
        // AWAY: do not trigger on TRV, otherwise same as
        // HOT/COLD: same (TRV trigger + thermostat trigger), target temp is changed by state change
        bool heat_required = false;
        if (current_mode != AWAY && TRV_requires_heat()) {
            heat_required = true;
        }
        if (thermostat_requires_heat()) {
            heat_required = true;
        }
        if (heat_required) {
            set_boiler(1);
        } else {
            set_boiler(0);
        }
    }

    static uint32_t last_log_publish_at = 0;
    if (logStr[0] != 0 && strcmp(logStr, lastLogStr) && millis() > last_log_publish_at + 5000) {
        mqtt.publish("boiler/log", logStr);
        strcpy(lastLogStr, logStr);
        logStr[0] = 0;
        last_log_publish_at = millis();
    }

    handle_change_mode();

    // Heartbeat
	if (millis() > send_next_ping_at) {
		send_next_ping_at = millis() + 10L * 60L * 1000L;
        mqtt.publish("boiler/available", "online", 1, true);
        mqtt_publish_TAstatus(boiler_val);
        mqtt.publish("boiler/mode", mode_names[current_mode]);
        mqtt_publish_target_temp();
	}
   
	delay(50); // needed to take advantage of modem sleep (70mA -> 50mA)
}
