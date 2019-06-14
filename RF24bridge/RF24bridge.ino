#include <RF24.h>
#include <SoftwareSerial.h>
#include "printf.h"

SoftwareSerial edfSerial(2, 6); //RX, TX (unused TX)


#define startFrame 0x02
#define endFrame 0x03

char teleinfo_buf[1024];
int teleinfo_cur = 0;

void send_frame()
{
    printf("\r\r\n%s\r\r\n", teleinfo_buf); // printable code for a frame: start and end with \r\r\n
}

void consume_teleinfo()
{
    static bool inFrame = false;

    while (edfSerial.available()) {
        char c = edfSerial.read() & 0x7F;
        if (c == startFrame) {
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
