/*
 * NAME: ServoPCA9685.cpp
 *
 * DESC: This library implements an interface given by the original Servo Library
 *   for Arduino to the implementation of the Adafruit-PWM-Servo-Driver-library
 *   which allows us to control 16 Servos via an I2C-bus controlled PCA9685-board
 *   as it is sold from Adafruit (https://www.adafruit.com/product/815).
 *
 * SOURCE: Code is available at https://github.com/ATrappmann/ServoPCA9685
 *
 * USES LIBRARIES:
 *  https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
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
#include "ServoPCA9685.h"

#define DEBUG 1
#include <TrappmannRobotics.h>

// Library for PCA9685 16-channel I2C servo driver
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver *ServoPCA9685::servoDriver = NULL;
Adafruit_PWMServoDriver *ServoPCA9685::initServoDriver(const uint8_t addr) {
  if (NULL == servoDriver) {
    servoDriver = new Adafruit_PWMServoDriver(addr);
    servoDriver->begin();
    servoDriver->setPWMFreq(SERVO_FREQ);
  }
  return servoDriver;
}

void ServoPCA9685::initDefaults() {
  this->lowerLimit = 0;
  this->upperLimit = 180;
  this->currentPulse = DEFAULT_PULSE_WIDTH;
}

ServoPCA9685::ServoPCA9685(const uint8_t addr) : GenericServo() {
  initServoDriver(addr);
  initDefaults();
}

ServoPCA9685::ServoPCA9685(const uint8_t pin, const uint8_t addr) : GenericServo(pin) {
  initServoDriver(addr);
  initDefaults();
}

uint8_t ServoPCA9685::attach(const uint8_t aPin) {
  SEROUT(F("Servo::attach(pin=") << aPin << ")\n");
  return this->attach(aPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t ServoPCA9685::attach(const uint8_t aPin, const uint16_t min, const uint16_t max) {
  SEROUT(F("Servo::attach(pin=") << aPin << ", minPulse=" << min << ", maxPulse=" << max << ")\n");
  if (aPin > 15) return INVALID_SERVO;
  if (min > max) return INVALID_SERVO;
  if (INVALID_SERVO != pin) detach();

  this->pin = aPin;
  this->minPulse = min;
  this->maxPulse = max;

  return pin;	// range is 0-15
}

void ServoPCA9685::write(uint8_t value) {
  SEROUT(F("Servo:write(") << value << F("), pin=") << pin << LF);
  // limit position
  value = constrain(value, lowerLimit, upperLimit);
  // map to pulse width
  uint16_t microseconds = map(value, 0, 180, minPulse, maxPulse);
  this->writeMicroseconds(microseconds);
}

void ServoPCA9685::writeMicroseconds(uint16_t value) {
  SEROUT(F("Servo:writeMicroseconds(") << value << F("), pin=") << pin << LF);
  // limit pulse length
  value = constrain(value, minPulse, maxPulse);
  this->currentPulse = value;
  if (INVALID_SERVO != pin) {
    servoDriver->writeMicroseconds(pin, value);
  }
}

void ServoPCA9685::writeAnalog(uint16_t value) {
  SEROUT(F("Servo:writeAnalog(") << value << F("), pin=") << pin << LF);
  // limit to 10-bit analog values
  if (value > 1023) value = 1023;
  uint16_t microseconds = map(value, 0, 1023, minPulse, maxPulse);
  this->writeMicroseconds(microseconds);
}

void ServoPCA9685::detach() {
  if (INVALID_SERVO == pin) return; // no servo attached
  servoDriver->setPin(pin, 0, false);
  pin = INVALID_SERVO;
}

uint8_t ServoPCA9685::read() {
  if (INVALID_SERVO == pin) return INVALID_SERVO; // no servo attached
  uint8_t position = map(currentPulse, minPulse, maxPulse, 0, 180);
  return position;
}

uint16_t ServoPCA9685::readMicroseconds() {
  if (INVALID_SERVO == pin) return INVALID_SERVO; // no servo attached
  return currentPulse;
}

bool ServoPCA9685::attached() const {
  if (INVALID_SERVO == pin) {
	   return false;
  }
  else return true;
}

void ServoPCA9685::limit(uint8_t low, uint8_t high) {
  if (high > 180) high = 180;
  if (low >= high) low = high;

  this->lowerLimit = low;
  this->upperLimit = high;
}
