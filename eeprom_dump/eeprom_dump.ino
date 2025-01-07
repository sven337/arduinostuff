#include <SPI.h>
#include "xmodem.h"

#define FLASH_CS 10
#define FLASH_SIZE 1048576UL  // 8Mbit = 1MB
    
long    File::size() {
      return FLASH_SIZE;
    }
    
int    File::read(uint8_t* buffer, size_t size) {
      for(size_t i = 0; i < size && position < this->size(); i++) {
        digitalWrite(FLASH_CS, LOW);
        SPI.transfer(0x03);  // Read command
        SPI.transfer((position >> 16) & 0xFF);
        SPI.transfer((position >> 8) & 0xFF);
        SPI.transfer(position & 0xFF);
        buffer[i] = SPI.transfer(0);
        digitalWrite(FLASH_CS, HIGH);
        position++;
      }
      return size;
    }
    
void    File::seek(long pos) {
      position = pos;
    }

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_CS, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
 
// Read and print chip ID
  while (1) {
      digitalWrite(FLASH_CS, LOW);
      SPI.transfer(0x9F);  // JEDEC ID command
      byte mf_id = SPI.transfer(0);
      byte mem_type = SPI.transfer(0);
      byte mem_cap = SPI.transfer(0);
      digitalWrite(FLASH_CS, HIGH);

      Serial.print("Received manufacturer ID");
      Serial.print(mf_id, HEX);
      Serial.print(" memory type:");
      Serial.print(mem_type, HEX);
      Serial.print(mem_cap, HEX);
      Serial.println();
      if (mem_type == 0x60 && mem_cap == 0x14) {
          break;
      }
  }

  Serial.println("Xmodem wait");
  File flash;
  
  // Send flash contents via XMODEM
    int XSendSub(File *f);
  XSendSub(&flash);
}

void loop() {
// Nothing to do here
}
