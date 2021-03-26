/*
 * NAME: ServoPCA9685.h
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
#ifndef SERVO_PCA9685_H
#define SERVO_PCA9685_H

#include "GenericServo.h"

#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define PULSE_RANGE           956
#define MIN_PULSE_WIDTH (DEFAULT_PULSE_WIDTH - PULSE_RANGE)   // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH (DEFAULT_PULSE_WIDTH + PULSE_RANGE)   // the longest pulse sent to a servo
#define SERVO_FREQ			       50	    // Hz

class Adafruit_PWMServoDriver;

class ServoPCA9685: public GenericServo
{
private:
  uint16_t  currentPulse; // current pulse width in microseconds
  uint16_t  minPulse;     // minimum pulse width in microseconds
  uint16_t	maxPulse;     // maximum pulse width in microseconds
  uint8_t 	lowerLimit;   // lower rotation limit for write(), default = 0
  uint8_t 	upperLimit;   // upper rotation limit for write(), default = 180

private:  // private initialization
      void initDefaults();

public:
  ServoPCA9685(const uint8_t addr = 0x40);
  ServoPCA9685(const uint8_t pin, const uint8_t addr);

  uint8_t attach(const uint8_t pin);      // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(const uint8_t pin, const uint16_t min, const uint16_t max); // as above but also sets min and max values for writes.
  void detach();
  void write(uint8_t value);              // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void writeAnalog(uint16_t value); // write corresponding pulse for value from ADC (range: 0-1023)
  void writeMicroseconds(uint16_t value); // Write pulse width in microseconds
  uint8_t read();                         // returns current pulse width as an angle between 0 and 180 degrees
  uint16_t readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached() const;                  // return true if this servo is attached, otherwise false

public:		// additional methods
  void limit(uint8_t low, uint8_t high);	// limit servo movement to specified range of degrees
  uint8_t getLowerLimit() const { return lowerLimit; }
  uint8_t getUpperLimit() const { return upperLimit; }

private:	// private methods
  static Adafruit_PWMServoDriver *initServoDriver(const uint8_t addr);
  static Adafruit_PWMServoDriver *servoDriver;
};

#endif /* SERVO_PCA9685_H */
