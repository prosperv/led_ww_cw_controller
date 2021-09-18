#include <Arduino.h>

#include "DelayRamp.h"

/* 
Inputs
Brightness - analog
ColorTemp - analog
*/

const static uint8_t brightnessInput = PIN_PA1;
const static uint8_t colorTemperatureInput = PIN_PA2;

const static uint8_t warmWhitePwmPin = PIN_PB0;
const static uint8_t coolWhitePwmPin = PIN_PB1;

const static uint8_t statusLedPin = PIN_PA6;

const static uint8_t stepSize = 8;
DelayRamp warmWhiteRamp(stepSize);
DelayRamp coolWhiteRamp(stepSize);

void setup() {
  Serial.begin(115200);
  Serial.println("Ready player one.");
  
  analogReference(INTERNAL4V34);
  
}

bool s = false;
void loop() {
  auto brightnessRaw = analogRead(brightnessInput);
  delay(5);
  auto colorTempRaw = analogRead(colorTemperatureInput);
  
  Serial.printf("brightnessRaw: %d, colorTempRaw: %d\r\n", brightnessRaw, colorTempRaw);

  float brightnessNormal = brightnessRaw / 1023.0f;
  float colorTempNormal = colorTempRaw / 1023.0f;

  int warmWhiteValue = colorTempNormal * brightnessNormal * 255;
  int coolWhiteValue = (1 - colorTempNormal) * brightnessNormal * 255;
  Serial.printf("warmWhiteValue: %d, coolWhiteValue: %d\r\n", warmWhiteValue, coolWhiteValue);

  warmWhiteRamp.setTarget(warmWhiteValue);
  coolWhiteRamp.setTarget(coolWhiteValue);
  
  int warmWhiteDelay = warmWhiteRamp.computeValue();
  int coolWhiteDelay = coolWhiteRamp.computeValue();
  Serial.printf("warmWhiteDelay: %d, coolWhiteDelay: %d\r\n", warmWhiteDelay, coolWhiteDelay);

  analogWrite(warmWhitePwmPin, warmWhiteDelay);
  analogWrite(coolWhitePwmPin, coolWhiteDelay);

  //Blink led
  digitalWrite(statusLedPin, s);
  s = !s;

  delay(200);
  Serial.println();
}