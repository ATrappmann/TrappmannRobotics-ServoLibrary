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
 *  https://github.com/arduino-libraries/Servo
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
#include <TrappmannRobotics/Debug.h>

// Library for PCA9685 16-channel I2C servo driver
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver *ServoPCA9685::servoDriver = NULL;
Adafruit_PWMServoDriver *ServoPCA9685::initServoDriver() {
  if (NULL == servoDriver) {
    servoDriver = new Adafruit_PWMServoDriver();
    servoDriver->begin();
    servoDriver->setPWMFreq(SERVO_FREQ);
  }
  return servoDriver;
}

ServoPCA9685::ServoPCA9685() {
  this->pin = INVALID_SERVO;
  this->lowerLimit = 0;
  this->upperLimit = 180;
}

uint8_t ServoPCA9685::attach(const uint8_t aPin) {
  SEROUT(F("Servo::attach(pin=") << aPin << ")\n");
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t ServoPCA9685::attach(const uint8_t aPin, int min, int max) {
  SEROUT(F("Servo::attach(pin=") << aPin << ", min=" << min << ", max=" << max)\n");
  if (min > max) return INVALID_SERVO;
  
  initServoDriver();
  
  this->pin = aPin;
  this->minPulse = min;
  this->maxPulse = max;
  
  return pin;	// range is 0-15
}

bool ServoPCA9685::write(const uint8_t aPos) {
  SEROUT(F("Servo:write(") << aPos << F("), pin=") << pin << LF);
  if (INVALID_SERVO == pin) return false; // no servo attached
  if (aPos > 180) return false; // out of range
  
  // limit position
  if (aPos < lowerLimit) aPos = lowerLimit;
  if (aPos > upperLimit) aPos = upperLimit;  
  this->position = aPos;
  
  uint16_t microseconds = map(position, 0, 180, minPulse, maxPulse);
  SEROUT("ms=" << microseconds << LF);
  servoDriver->writeMicroseconds(pin, microseconds);
  return true;
}

void ServoPCA9685::detach() {
  if (INVALID_SERVO == pin) return; // no servo attached
  servoDriver->setPin(pin, 0, false);
}

uint8_t ServoPCA9685::read() const {
  if (INVALID_SERVO == pin) return INVALID_SERVO; // no servo attached
  return position;
}

uint16_t ServoPCA9685::readMicroseconds() const {
  if (INVALID_SERVO == pin) return INVALID_SERVO; // no servo attached
  return map(position, 0, 180, minPulse, maxPulse);
}

bool ServoPCA9685::attached() {
  if (INVALID_SERVO == pin) {
	return false;
  }
  else return true;
}

void ServoPCA9685::limit(const uint8_t low, const uint8_t high) {
  if (high > 180) return;
  if (low >= high) return;
  
  this->lowerLimit = low;
  this->upperLimit = high;
}
