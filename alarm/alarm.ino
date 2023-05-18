#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include "wifi_params.h"
#include "mqtt_login.h"

/** Surveillance de détecteurs d'ouverture d'alarme + détecteur de fumée et de CO.
  Les détecteurs d'ouverture étaient cablés sur une Honeywell Galaxy 48 qui est tombée en panne. Ils sont connectés avec des résistances de telle sorte que (normally) closed = 1kohm , open = 2kohm.
  Use an analog multiplexer on a ESP8266 to check for those resistor values.

  The method is to apply a 3.3V voltage through a known resistor Rk = 1kOhm, and mesure voltage using an ADC (external to the ESP8266 because it's easier). 
  Let E = 3.3V, Rk = 1kOhm, R is the resistance we're looking for, Ur is the voltage across it = the voltage measured by the ADC.
  We have : R=Ur.Rk/(E-Ur)
  **/


#define ARR_SZ(X) sizeof(X)/sizeof(X[0])

ESP8266WebServer websrv(80);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "192.168.1.6", 1883, MQTT_USER, MQTT_PASS);

enum detector_status {
    OPENCIRCUIT = 0,
    NORMAL = 1,
    ALARM = 2,
    CLOSEDCIRCUIT = 3,
    UNKNOWN = 4,
};

struct {
    const char *name;
    enum detector_status status;
    enum detector_status oldstatus;
} sensors[] = {
               { "garage", UNKNOWN, UNKNOWN }, 
               { "sdb", UNKNOWN, UNKNOWN }, 
               { "chfond", UNKNOWN, UNKNOWN }, 
               { "chmilieu", UNKNOWN, UNKNOWN }, 
               { "bureau", UNKNOWN, UNKNOWN }, 
               { "maindoor", UNKNOWN, UNKNOWN }, 
               { "baievitreeG", UNKNOWN, UNKNOWN }, 
               { "baievitreeD", UNKNOWN, UNKNOWN }, 
               { "fenetre_sejour", UNKNOWN, UNKNOWN }, 
               { "cuisine", UNKNOWN, UNKNOWN }, 
               { "smoke", UNKNOWN, UNKNOWN }, 
               { "carbonmonox", UNKNOWN, UNKNOWN }, 
              };

const char *status_str[] = { [0] = "opencircuit", 
                             [1] = "normal",
                             [2] = "alarm",
                             [3] = "closedcircuit",
                             [4] = "unknown" };

unsigned long int send_fullstatus_at = 5000;

/*  MCP3201 ADC Hardware:
      MCP3201 Pin   ---------------- ESP8266 Pin
-       1-VREF      ----------------  3,3V
-       2-IN+       ----------------  ANALOG SIGNAL +
-       3-IN-       ----------------  ANALOG SIGNAL -
-       4-GND       ----------------  GND
-       5-CS        ----CS----------  GPIO15/CS (PIN 19)
-       6-Dout(MISO)----MISO--------  GPIO12/MISO (PIN 16)
-       7-CLK       ----SCLK--------  GPIO14 (PIN 17)
-       8-VDD       ----------------  3.3V
*/
static inline void setDataBits(uint16_t bits) {
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    bits--;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

#ifndef ICACHE_RAM_ATTR
#warning "Should have ICACHE_RAM_ATTR"
#define ICACHE_RAM_ATTR     __attribute__((section(".iram.text")))
#endif

/* SPI code based on the SPI library */
static inline ICACHE_RAM_ATTR uint16_t transfer16(void) {
	union {
		uint16_t val;
		struct {
			uint8_t lsb;
			uint8_t msb;
		};
	} out;


	// Transfer 16 bits at once, leaving HW CS low for the whole 16 bits 
	while(SPI1CMD & SPIBUSY) {}
	SPI1W0 = 0;
	SPI1CMD |= SPIBUSY;
	while(SPI1CMD & SPIBUSY) {}

	/* Follow MCP3201's datasheet: return value looks like this:
	xxxBA987 65432101
	We want 
	76543210 0000BA98

	So swap the bytes, select 12 bits starting at bit 1, and shift right by one.
	*/

	out.val = SPI1W0 & 0xFFFF;
	uint8_t tmp = out.msb;
	out.msb = out.lsb;
	out.lsb = tmp;

	out.val &= (0x0FFF << 1);
	out.val >>= 1;
	return out.val;
}

void spiBegin(void) 
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
  SPI.setHwCs(1);
  setDataBits(16);
}


char tempBuf[4096];
static void handleRoot() {
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf(tempBuf, 1024,"<html><head><title>Alarm control</title></head><body>\
    <h1>Alarm control</h1>\
    <p>Built on %s at %s</p><p>Uptime: %02d:%02d:%02d</p><br/><br/>"\
    "<table><thead><tr><th>Sensor</th><th>Status</th></tr></thead><tbody>",
		__DATE__, __TIME__, hr, min % 60, sec % 60);

    unsigned int i;
    for (i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++) {
        sprintf(tempBuf+strlen(tempBuf), "<tr><td>%s</td><td>%s</td></tr>", sensors[i].name, status_str[sensors[i].status]);
    }
    strcat(tempBuf, "</tbody></table></body></html>");
	websrv.send(200, "text/html", tempBuf);
}

void ota_onstart(void)
{
}

void ota_onprogress(unsigned int sz, unsigned int total)
{
	Serial.print("OTA: "); Serial.print(sz); Serial.print("/"); Serial.print(total);
	Serial.print("="); Serial.print(100*sz/total); Serial.println("%%");
}

void ota_onerror(ota_error_t err)
{
	Serial.print("OTA ERROR:"); Serial.println((int)err);
}

void setup ( void ) {
	Serial.begin(115200);

	Serial.println(ESP.getResetReason());
	WiFi.mode(WIFI_STA);
	WiFi.setOutputPower(10); // reduce power to 10dBm = 10mW

	WiFi.begin(ssid, password);

	WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
	Serial.println ( "" );
	Serial.print("Connecting to ");
	Serial.print(ssid); Serial.print(" ");

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

	websrv.begin();

	ArduinoOTA.onStart(ota_onstart);
	ArduinoOTA.onError(ota_onerror);
	ArduinoOTA.onProgress(ota_onprogress);
	ArduinoOTA.setHostname("alarm-control");
	ArduinoOTA.begin();

    // Selection bits for CD74HC4067 multiplexer
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);


    spiBegin();
}

uint32_t measure_resistance(uint8_t input_index)
{
    const float E = 3.3;
    const float Rk = 1000;

    int d0val = input_index & 0x1; 
    int d1val = (input_index >> 1) & 0x1;
    int d2val = (input_index >> 2) & 0x1;
    int d3val = (input_index >> 3) & 0x1;
    digitalWrite(D0, d0val);
    digitalWrite(D1, d1val);
    digitalWrite(D2, d2val);
    digitalWrite(D3, d3val);
    delay(10);

    uint16_t adcval = transfer16(); // linear 0-4096 for 0V-3.3V
    float Ur = (adcval * 3.3) / 4096.0;
  
    // R=Ur.Rk/(E-Ur) with Ur = adcval * 3.3 / 4096.0

    uint32_t R = (uint32_t)(Ur * Rk / (E - Ur));
    return R;
}

enum detector_status status_from_res(uint32_t res)
{
    if (res > 1600 && res < 5000) {
        return ALARM;
    } else if (res > 800 && res < 1300) {
        return NORMAL;
    } else if (res < 800) {
        return CLOSEDCIRCUIT;
    } else if (res > 5000) {
        return OPENCIRCUIT;
    }

    return UNKNOWN;
}

void send_sensor_update(uint8_t idx)
{
    // HTTP
    HTTPClient http;
    char URI[200];

    sprintf(URI, "http://192.168.1.6:5000/update/alarm/%s/%s", sensors[idx].name, status_str[sensors[idx].status]);
    Serial.print(URI);

    http.begin(URI);

    int httpCode = http.GET();

    // httpCode will be negative on error
    if(httpCode) {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    }

    http.end();

    // MQTT
    if (!mqtt.connected()) {
        Serial.print("Connecting MQTT...");
        mqtt.connect();
        Serial.println("done");
    }
    if (!mqtt.connected()) {
        // Ouch
        return;
    }
    sprintf(URI, "alarm/%s", sensors[idx].name);
    mqtt.publish(URI, status_str[sensors[idx].status], 1);
}

void loop ( void ) {
	ArduinoOTA.handle();
	websrv.handleClient();

    int i;
    for (i = 0; i < ARR_SZ(sensors); i++) {
        uint32_t res = measure_resistance(i);
        sensors[i].status = status_from_res(res);
        if (sensors[i].status != sensors[i].oldstatus) {
            // Report modified status
            // XXX
            printf("Sensor %d changed status from %s to %s res %d\r\n", i, status_str[sensors[i].oldstatus], status_str[sensors[i].status], res);
            sensors[i].oldstatus = sensors[i].status;
            send_sensor_update(i);
        }
    }

/*	if (millis() > send_fullstatus_at) {
		send_fullstatus_at = millis() + 15L * 60L * 1000L; // 15 minutes

        // Report full status
        for (i = 0; i < ARR_SZ(sensors); i++) {
            send_sensor_update(i);
        }
	
	}*/
   
	delay(500);
}
