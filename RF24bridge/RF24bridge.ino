#include <RF24.h>
#include <SoftwareSerial.h>
#include "printf.h"

SoftwareSerial edfSerial(2, 6); //RX, TX (unused TX)


// teleinfo code from https://blog.antoineve.me/2015/05/30/teleinformation-erdf-avec-arduino/
#define startFrame 0x02
#define endFrame 0x03
#define startLine 0x0A
#define endLine 0x0D

char teleinfo_buf[1024];
int teleinfo_cur = 0;

void send_frame()
{
    printf("Got frame\r\n%s\r\n", teleinfo_buf);
}

void consume_teleinfo()
{
    static bool inFrame = false;

    while (edfSerial.available()) {
        char c = edfSerial.read() & 0x7F;
        if (c == startFrame) {
            printf("Starting frame\r\n");
            inFrame = true;
            teleinfo_cur = 0;
            teleinfo_buf[teleinfo_cur] = 0;
            continue;
        } else if (c == endFrame) {
            inFrame = false;
            teleinfo_buf[teleinfo_cur] = 0;
            teleinfo_cur = 0;
            send_frame();
            continue;
        }

        if (inFrame) {
            teleinfo_buf[teleinfo_cur++] = c;
        }
    }
}

void setup()
{
    printf_begin();
    Serial.begin(115200);
    printf("Hello\r\n");
    edfSerial.begin(1200); 

}

void loop()
{
    if (edfSerial.available()) {
        consume_teleinfo();
    }

    if (Serial.available()) {
    }
}
