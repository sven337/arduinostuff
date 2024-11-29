#include <avr/sleep.h>
#include <avr/interrupt.h>

const uint8_t MOTOR_PIN = 10;
const uint8_t LED_PIN = 11;
const uint8_t BTN_PLUS = 12;
const uint8_t BTN_MINUS = 9;
const uint8_t PWR_BTN = 13;

volatile bool isOn = false;
uint8_t currentMode = 0;
uint8_t maxPWM = 192;  // 75% of 255
uint32_t lastButtonPress = 0;
const uint16_t LONG_PRESS = 1000;
const uint8_t NUM_MODES = 4;

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PLUS, INPUT_PULLUP);
  pinMode(BTN_MINUS, INPUT_PULLUP);
  pinMode(PWR_BTN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PWR_BTN), wakeUpISR, FALLING);
  enterSleep();
}

void loop() {
  if (!isOn) {
    enterSleep();
    return;
  }

  handleButtons();
  updateVibration();
}

void handleButtons() {
  static uint32_t lastDebounceTime = 0;
  const uint16_t debounceDelay = 50;

  if (digitalRead(PWR_BTN) == LOW) {
    if (millis() - lastButtonPress > debounceDelay) {
      if (millis() - lastButtonPress > LONG_PRESS) {
        isOn = false;
        analogWrite(MOTOR_PIN, 0);
        analogWrite(LED_PIN, 255);  // LED off (inverted)
      }
    }
  } else {
    if (millis() - lastButtonPress > debounceDelay && 
        millis() - lastButtonPress < LONG_PRESS) {
      currentMode = (currentMode + 1) % NUM_MODES;
    }
    lastButtonPress = millis();
  }

  if (digitalRead(BTN_PLUS) == LOW && maxPWM < 255) {
    maxPWM += 5;
  }
  if (digitalRead(BTN_MINUS) == LOW && maxPWM > 0) {
    maxPWM -= 5;
  }
}

void updateVibration() {
  static uint32_t lastUpdate = 0;
  static uint8_t currentPWM = 0;
  static bool rampingUp = true;

  switch (currentMode) {
    case 0:  // Constant
      analogWrite(MOTOR_PIN, maxPWM);
      analogWrite(LED_PIN, 255 - maxPWM);
      break;
      
    case 1:  // Pulse (fast on/off)
      if (millis() - lastUpdate > 100) {
        currentPWM = currentPWM > 0 ? 0 : maxPWM;
        analogWrite(MOTOR_PIN, currentPWM);
        analogWrite(LED_PIN, 255 - currentPWM);
        lastUpdate = millis();
      }
      break;
      
    case 2:  // Slow pulse
      if (millis() - lastUpdate > 500) {
        currentPWM = currentPWM > 0 ? 0 : maxPWM;
        analogWrite(MOTOR_PIN, currentPWM);
        analogWrite(LED_PIN, 255 - currentPWM);
        lastUpdate = millis();
      }
      break;
      
    case 3:  // Ramp
      if (millis() - lastUpdate > 20) {
        if (rampingUp) {
          currentPWM = min(currentPWM + 5, maxPWM);
          if (currentPWM >= maxPWM) rampingUp = false;
        } else {
          currentPWM = max(currentPWM - 5, 0);
          if (currentPWM == 0) rampingUp = true;
        }
        analogWrite(MOTOR_PIN, currentPWM);
        analogWrite(LED_PIN, 255 - currentPWM);
        lastUpdate = millis();
      }
      break;
  }
}

void enterSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}

void wakeUpISR() {
  if (!isOn) {
    isOn = true;
    currentMode = 0;
    lastButtonPress = millis();
  }
}

