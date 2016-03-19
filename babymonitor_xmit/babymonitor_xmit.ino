// Based on ESP_MCP3201_SPI
/*  Hardware: 
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


#include <SPI.h>
 
// Pin definitions: 
const int scePin = 15;   	// SCE - Chip select
/* HW definition of alternate function:
static const uint8_t MOSI  = 13;
static const uint8_t MISO  = 12;
static const uint8_t SCK   = 14;
*/

uint16_t adc_buf[2][25];
int current_adc_buf;
int adc_buf_pos;
int send_samples_now;

void mcp_output(uint16_t out)
{
  uint16_t temp=0;
  temp = out & 0xFF00;   
  temp = temp >> 8;      //LSB
  out = out << 8;        //MSB
  out = out + temp;
  out = out >> 1;
  out = out & 0x0FFF;
  Serial.print(out);
  Serial.print(" ");
  Serial.println(millis());
}


void spiBegin(void) 
{
  pinMode(scePin, OUTPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
  digitalWrite(scePin, HIGH);
}

void sample_isr(void)
{
	// Read a sample from ADC
	digitalWrite(scePin, LOW);
	adc_buf[current_adc_buf][adc_buf_pos] = SPI.transfer16(0x00);
	digitalWrite(scePin, HIGH);

	adc_buf_pos++;
	if (adc_buf_pos > sizeof(adc_buf[0])/sizeof(adc_buf[0][0])) {
		adc_buf_pos = 0;
		current_adc_buf = !current_adc_buf;
		send_samples_now = 1;
	}
}
 
void setup(void)
{ 
	Serial.begin(115200);
	spiBegin(); 

	timer1_isr_init();
	timer1_attachInterrupt(sample_isr);
	timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
	timer1_write(clockCyclesPerMicrosecond() / 16 * 125);
	Serial.println("setup done");
}

 
void loop() 
{
  if (send_samples_now) {
	  Serial.print("Output at ");
	  Serial.println(millis());
	  send_samples_now = 0;
  }	  
}

