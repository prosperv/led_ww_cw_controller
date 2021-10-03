#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "DelayRamp.h"
#include "Hysteresis.h"

// #define DEBUG
#ifdef DEBUG
#define BEGIN(x) Serial.begin(x)
#define PRINTLN(x) Serial.println(x)
#define PRINT(x) Serial.print(x)
#define PRINTF(...) Serial.printf(__VA_ARGS__)

#else
#define BEGIN(x)
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

const static uint8_t stepSize = 1;
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
  // RTC.PITCTRLA |= RTC_PITEN_bm;

  // Set period/frequency of interrupt and enable PIT
  // 0x1 = 4 cycles f=256 Hz
  // 0x2 = 8 cycles f=128 Hz
  // 0x3 = 16 cycles f=64 Hz
  // 0x4 = 32 cycles f=32 Hz
  // 0x5 = 64 cycles f=16 Hz
  // 0x6 = 128 cycles f=8 Hz
  #ifdef DEBUG
  RTC.PITCTRLA |= RTC_PERIOD_CYC128_gc | RTC_PITEN_bm;
  #else
  RTC.PITCTRLA = RTC_PERIOD_CYC8_gc | RTC_PITEN_bm;
  #endif

  // Clear interrupt flag
  RTC.PITINTFLAGS = 0x1;
}

void setup() {
  wdt_enable(WDTO_2S);
  BEGIN(115200);
  PRINTLN("\n\rReady player one.\n\r");
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
  PRINT("Sleeping... ");
  // Set sleep mode and enable sleep.
  SLPCTRL.CTRLA = SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm;

  // Serial output will wake the cpu so make sure we disable any prints
  do {
    sleep_cpu();
  } while (SLPCTRL.CTRLA & SLPCTRL_SEN_bm);
  PRINTLN(" Awake");
}

void pwmWrite(const uint8_t& pin, int val)
{
  if (val <= 0)
  {
    val = 0;
  }
  else if (val >= 255) {
    val = 255;
  }

  analogWrite(pin, val);
}

int warmWhitePreviousValue = 0;
int coolWhitePreviousValue = 0;

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

  if (warmWhiteDelay != warmWhitePreviousValue)
  pwmWrite(warmWhitePwmPin, warmWhiteDelay);

  if (coolWhiteDelay != coolWhitePreviousValue)
  pwmWrite(coolWhitePwmPin, coolWhiteDelay);

  warmWhitePreviousValue = warmWhiteDelay;
  coolWhitePreviousValue = coolWhiteDelay;
  idleSleep();

  PRINTLN();
}