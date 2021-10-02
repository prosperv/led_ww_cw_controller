#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "DelayRamp.h"
#include "Hysteresis.h"

// #define DEBUG
#ifdef DEBUG
#define PRINTLN(x) Serial.println(x)
#define PRINT(x) Serial.print(x)
#define PRINTF(...) Serial.printf(__VA_ARGS__)

#else
#define PRINTLN(...)
#define PRINT(...)
#define PRINTF(...)
#endif
const static uint8_t brightnessInput = PIN_PA1;
const static uint8_t colorTemperatureInput = PIN_PA2;

const static uint8_t warmWhitePwmPin = PIN_PB0;
const static uint8_t coolWhitePwmPin = PIN_PB1;

const static uint8_t powerSense = PIN_PA4; // Unused

const static uint8_t statusLedPin = PIN_PA6;

const static uint8_t stepSize = 8;
DelayRamp warmWhiteRamp(stepSize);
DelayRamp coolWhiteRamp(stepSize);

const static uint8_t hysteresisThreshold = 5;
Hysteresis brightnessHystersis(hysteresisThreshold);
Hysteresis colorTempHystersis(hysteresisThreshold);

void setupPeriodicInterruptTimer(){
  // Select 1024Hz clock for RTC/PIT
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;

  //Enable interrupt
  RTC.PITINTCTRL = RTC_PI_bm;

  //Enable PIT module
  // RTC.PITCTRLA |= 0x1;

  // Set period/frequency of interrupt
  // 0x1 = 4 cycles f=256 Hz
  // 0x2 = 8 cycles f=128 Hz
  // 0x3 = 16 cycles f=64 Hz
  // 0x4 = 32 cycles f=32 Hz
  // 0x4 = 64 cycles f=16 Hz
  // 0x4 = 128 cycles f=8 Hz
  // RTC.PITCTRLA |= RTC_PERIOD_CYC8_gc;
  RTC.PITCTRLA = RTC_PERIOD_CYC512_gc | RTC_PITEN_bm;

  // Clear interrupt flag
  RTC.PITINTFLAGS = 0x1;
}

void setup() {
  wdt_enable(WDTO_2S);
  #ifdef DEBUG
  Serial.begin(115200);
  PRINTLN("\n\rReady player one.\n\r");
  #endif
  setupPeriodicInterruptTimer(); 

  analogReference(INTERNAL4V34);
  interrupts();
}

bool led = false;
ISR(RTC_PIT_vect) {
  sleep_disable();
  digitalWrite(statusLedPin, led);
  led = !led;
  RTC.PITINTFLAGS = 0x1;
}


void idleSleep() {
  Serial.print("Sleeping... ");
  // Set sleep mode and enable sleep.
  SLPCTRL.CTRLA = SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm;

  // Serial output will wake the cpu so make sure we disable any prints
  do {
    sleep_cpu();
  } while (SLPCTRL.CTRLA & SLPCTRL_SEN_bm);
  PRINTLN(" Awake");
}

void loop() {
  wdt_reset();
  auto brightnessRaw = analogRead(brightnessInput);
  delay(1);
  auto colorTempRaw = analogRead(colorTemperatureInput);

  PRINTF("brightnessRaw: %d, colorTempRaw: %d\r\n", brightnessRaw, colorTempRaw);

  auto brightnessHystersisValue = brightnessHystersis.process(brightnessRaw);
  auto colorTempHystersisValue = colorTempHystersis.process(colorTempRaw);

  PRINTF("brightnessHystersisValue: %d, colorTempHystersisValue: %d\r\n", brightnessHystersisValue, colorTempHystersisValue);

  float brightnessNormal = brightnessHystersisValue / 1023.0f;
  float colorTempNormal = colorTempHystersisValue / 1023.0f;

  int warmWhiteValue = colorTempNormal * brightnessNormal * 255;
  int coolWhiteValue = (1 - colorTempNormal) * brightnessNormal * 255;
  PRINTF("warmWhiteValue: %d, coolWhiteValue: %d\r\n", warmWhiteValue, coolWhiteValue);

  warmWhiteRamp.setTarget(warmWhiteValue);
  coolWhiteRamp.setTarget(coolWhiteValue);
  
  int warmWhiteDelay = warmWhiteRamp.computeValue();
  int coolWhiteDelay = coolWhiteRamp.computeValue();
  PRINTF("warmWhiteDelay: %d, coolWhiteDelay: %d\r\n", warmWhiteDelay, coolWhiteDelay);

  analogWrite(warmWhitePwmPin, warmWhiteDelay);
  analogWrite(coolWhitePwmPin, coolWhiteDelay);

  idleSleep();

  PRINTLN();
}