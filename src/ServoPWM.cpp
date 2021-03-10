/*
 * NAME: ServoPWM.cpp
 *
 * DESC: This library implements an interface given by the original Servo Library
 *   for Arduino with additional functions.
 *
 * SOURCE: Code is available at https://github.com/ATrappmann/ServoPWM
 *
 * USES LIBRARIES:
 *  https://github.com/arduino-libraries/Servo
 *
 * MIT License
 *
 * Copyright (c) 2021 Andreas Trappmann
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "ServoPWM.h"

#define DEBUG 1
#include <TrappmannRobotics.h>
#include <Arduino.h>

ServoPWM::ServoPWM() {
  this->lowerLimit = 0;
  this->upperLimit = 180;
}

uint8_t ServoPWM::attach(const uint8_t aPin) {
  SEROUT(F("Servo::attach(pin=") << aPin << ")\n");
  return this->attach(aPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t ServoPWM::attach(const uint8_t aPin, const uint16_t min, const uint16_t max) {
  SEROUT(F("Servo::attach(pin=") << aPin << ", minPulse=" << min << ", maxPulse=" << max << ")\n");
  if (min > max) return INVALID_SERVO;

  this->minPulse = min;
  this->maxPulse = max;

  return servo.attach(aPin, min, max);
}

void ServoPWM::write(uint8_t value) {
  SEROUT(F("Servo:write(") << value << F(")\n"));
  // limit position
  if (value < lowerLimit) value = lowerLimit;
  if (value > upperLimit) value = upperLimit;
  // map to pulse width
  uint16_t microseconds = map(value, 0, 180, minPulse, maxPulse);
  this->writeMicroseconds(microseconds);
}

void ServoPWM::writeMicroseconds(uint16_t value) {
  SEROUT(F("Servo:writeMicroseconds(") << value << F(")\n"));
  // limit pulse length
  if (value < minPulse) value = minPulse;
  if (value > maxPulse) value = maxPulse;
  servo.writeMicroseconds(value);
}

void ServoPWM::writeAnalog(uint16_t value) {
  SEROUT(F("Servo:writeAnalog(") << value << F(")\n"));
  // limit to 10-bit analog values
  if (value > 1023) value = 1023;
  uint16_t microseconds = map(value, 0, 1023, minPulse, maxPulse);
  this->writeMicroseconds(microseconds);
}

void ServoPWM::detach() {
  servo.detach();
}

uint8_t ServoPWM::read() {
  return servo.read();
}

uint16_t ServoPWM::readMicroseconds() {
  return servo.readMicroseconds();
}

bool ServoPWM::attached() const {
  return servo.attached();
}

void ServoPWM::limit(uint8_t low, uint8_t high) {
  if (high > 180) high = 180;
  if (low >= high) low = high;

  this->lowerLimit = low;
  this->upperLimit = high;
}
