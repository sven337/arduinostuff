#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <PinChangeInterrupt.h>

const uint8_t MOTOR_PIN = 10;
const uint8_t LED_PIN = 11; // Both the battery charging circuit and this MCU try to drive this pin...
const uint8_t PWR_BTN = 12; // not ideal, should use an interrupt-capable pin
const uint8_t BTN_PLUS = 9;
const uint8_t BTN_MINUS = 13;

uint8_t maxPWM = 192;  // 75% of 255
uint32_t lastButtonPress = 0;
const uint16_t LONG_PRESS = 750;

const uint8_t MIN_PWM = 65; // minimum effective PWM value
uint8_t currentPWM = 0;

bool ignorePowerPress = false;
  
uint32_t lastPatternUpdate = 0;

enum mode {
    OFF = 0,
    CONSTANT = 1,
    FAST_PULSE = 2,
    SLOW_PULSE = 3,
    RAMP = 4,
    ADJUSTABLE_PULSE = 5,
    PULSETRAIN = 6,

    NUM_MODES,
} currentMode;

void handleButtons() {
    static bool powerBtnState = HIGH;
    static bool plusBtnState = HIGH;
    static bool minusBtnState = HIGH;
    const uint16_t debounceDelay = 50;
    static uint32_t plusButtonPress = 0;
    static uint32_t minusButtonPress = 0;

    // Read current button states
    bool powerBtnCurrent = digitalRead(PWR_BTN);
    bool plusBtnCurrent = digitalRead(BTN_PLUS);
    bool minusBtnCurrent = digitalRead(BTN_MINUS);

    //  Hack because the interaction between short+long press logic here and
    //  the ISR is painful. The ISR sets a simple flag and this inhibits
    //  everything here except turn on the device.
    if (ignorePowerPress) {
        if (!powerBtnCurrent) {
            Serial.println("inhbit until power released");
            return;
        }
        ignorePowerPress = false;
    }


     // Power button handling
    if (powerBtnCurrent != powerBtnState) {
        if (powerBtnCurrent == LOW) {  // Button just pressed
            lastButtonPress = millis();
        }
        powerBtnState = powerBtnCurrent;
    
        uint16_t btnPressedFor = millis() - lastButtonPress;
        if (btnPressedFor > debounceDelay) {
            if (btnPressedFor < LONG_PRESS) {
                currentMode = (currentMode + 1) % NUM_MODES;
                Serial.print("Mode changed to: ");
                Serial.println(currentMode);
            }
        }

    }

    if (powerBtnCurrent == powerBtnState && powerBtnCurrent == 0) {
        // Pressed power button, handle long press
        uint16_t btnPressedFor = millis() - lastButtonPress;
        if (btnPressedFor > LONG_PRESS) {
            Serial.println("Power long press - turning off");
            digitalWrite(MOTOR_PIN, 0);
            currentMode = OFF;
        }
    }

 // Plus button handling
    if (plusBtnCurrent != plusBtnState) {
        if (plusBtnCurrent == LOW) {
            plusButtonPress = millis();
        } else if (millis() - plusButtonPress > debounceDelay && millis() - plusButtonPress < LONG_PRESS) {
            if (maxPWM <= 250) maxPWM += 5;
            else maxPWM = 255;
            Serial.print("Power increased to: ");
            Serial.println(maxPWM);
        }
        plusBtnState = plusBtnCurrent;
    }

    if (plusBtnCurrent == plusBtnState && plusBtnCurrent == LOW) {
        if (millis() - plusButtonPress > LONG_PRESS) {
            maxPWM = 255;
            Serial.println("Max power set to 255");
        }
    }

    // Minus button handling
    if (minusBtnCurrent != minusBtnState) {
        if (minusBtnCurrent == LOW) {
            minusButtonPress = millis();
        } else if (millis() - minusButtonPress > debounceDelay && millis() - minusButtonPress < LONG_PRESS) {
            if (maxPWM >= MIN_PWM) maxPWM -= 5;
            else maxPWM = MIN_PWM;
            Serial.print("Power decreased to: ");
            Serial.println(maxPWM);
        }
        minusBtnState = minusBtnCurrent;
    }

    if (minusBtnCurrent == minusBtnState && minusBtnCurrent == LOW) {
        if (millis() - minusButtonPress > LONG_PRESS) {
            maxPWM = MIN_PWM;
            Serial.println("Min power set");
        }
    }
}

void pulseUpdate(uint16_t period, bool maxSpeed)
{
    uint8_t pwm = maxPWM;
    if (maxSpeed) {
        pwm = 255;
    }
    if ((millis() - lastPatternUpdate) > period) {
        currentPWM = currentPWM > 0 ? 0 : pwm;
        Serial.println(currentPWM);
        analogWrite(MOTOR_PIN, currentPWM);
        analogWrite(LED_PIN, 255 - currentPWM);
        lastPatternUpdate = millis();
    }
}

void updateVibration() {
  static bool rampingUp = true;
  static uint32_t holdingRampTill = 0;;
  static bool inBurstPhase = true;
  static int pulseCount = 0;

  switch (currentMode) {
    case OFF:
      digitalWrite(MOTOR_PIN, 0);
      digitalWrite(LED_PIN, 1);
      break;

    case CONSTANT:
      analogWrite(MOTOR_PIN, maxPWM);
      digitalWrite(LED_PIN, 0);
      break;
      
    case FAST_PULSE:
      pulseUpdate(75, false);
      break;
      
    case SLOW_PULSE:
      pulseUpdate(250, false);
      break;
    
    case ADJUSTABLE_PULSE: 
      // Adjust pulse *speed* with PWM settings, at max PWM
      pulseUpdate(map(maxPWM, 255, MIN_PWM, 10, 400), true);
      break;
      
    case RAMP:  // Ramp
      if (millis() - lastPatternUpdate > 20) {
          if (millis() >= holdingRampTill) {
              if (rampingUp) {
                  currentPWM = min(currentPWM + 5, maxPWM);
                  if (currentPWM >= maxPWM) {
                      // Hold at min or max for a random amount of time. Reducing
                      // predictibility creates anticipation and frustration ;)
                      holdingRampTill = millis() + random(0, 3000);
                      rampingUp = false;
                  }
              } else {
                  currentPWM = max(currentPWM - 5, 0);
                  if (currentPWM <= MIN_PWM) {
                      currentPWM = MIN_PWM;
                      holdingRampTill = millis() + random(0, 1000);
                      rampingUp = true;
                  }
              }
              analogWrite(MOTOR_PIN, currentPWM);
              analogWrite(LED_PIN, 255 - currentPWM);
              lastPatternUpdate = millis();
              Serial.println(currentPWM);
          }
      }
      break;
    case PULSETRAIN: // short pulses followed by a continuous wave
        if (inBurstPhase) {
                if (millis() - lastPatternUpdate > 100) {
                    currentPWM = currentPWM > 0 ? 0 : maxPWM;
                    analogWrite(MOTOR_PIN, currentPWM);
                    analogWrite(LED_PIN, 255 - currentPWM);
                    lastPatternUpdate = millis();
                    
                    if (currentPWM == 0) {
                        pulseCount++;
                        if (pulseCount >= 10) {
                            pulseCount = 0;
                            inBurstPhase = false;
                            lastPatternUpdate = millis();
                        }
                    }
                }
            } else {
                analogWrite(MOTOR_PIN, maxPWM);
                analogWrite(LED_PIN, 255 - maxPWM);
                if (millis() - lastPatternUpdate > 2000) {
                    inBurstPhase = true;
                    lastPatternUpdate = millis();
                }
            }
      break;
  }

}

void enterSleep() {
    digitalWrite(LED_PIN, 1);
    pinMode(LED_PIN, INPUT);
    Serial.println("going to sleep");
    delay(500);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void powerBtnISR() {
    Serial.println("PwrBTN ISR");
    if (currentMode == OFF) {
        currentMode = CONSTANT;
        pinMode(LED_PIN, OUTPUT);
        Serial.println("Turning on");
        ignorePowerPress = true;
    }
}

void loop() {
  handleButtons();
  updateVibration();
  if (currentMode == OFF) {
      enterSleep();
  }
}

void setup() {
  Serial.begin(115200);

  // Timer 2 frequencies for PWM
/*TCCR2B = TCCR2B & B11111000 | B00000001; // 31372.55 Hz
TCCR2B = TCCR2B & B11111000 | B00000010; // 3921.16 Hz
TCCR2B = TCCR2B & B11111000 | B00000011; // 980.39 Hz*/
TCCR2B = TCCR2B & B11111000 | B00000100; // 490.20 Hz (default)
//TCCR2B = TCCR2B & B11111000 | B00000101; // 245.10 Hz

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PLUS, INPUT_PULLUP);
  pinMode(BTN_MINUS, INPUT_PULLUP);
  pinMode(PWR_BTN, INPUT_PULLUP);

  digitalWrite(MOTOR_PIN, 0);
  digitalWrite(LED_PIN, 1);
  
  Serial.println("Device initialized");
 
  attachPCINT(digitalPinToPCINT(PWR_BTN), powerBtnISR, FALLING);
  enterSleep();
}
