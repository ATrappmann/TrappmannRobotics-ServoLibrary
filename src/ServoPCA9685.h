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

/*
  Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2009 Michael Margolis.  All right reserved.

  A servo is activated by creating an instance of the Servo class passing
  the desired pin to the attach() method.
  The servos are pulsed in the background using the value most recently
  written using the write() method.

  The methods are:

    Servo - Class for manipulating servo motors connected to Arduino pins.

    attach(pin )  - Attaches a servo motor to an i/o pin.
    attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
    default min is 544, max is 2400

    write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
    writeMicroseconds() - Sets the servo pulse width in microseconds
    read()      - Gets the last written servo pulse width as an angle between 0 and 180.
    readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
    attached()  - Returns true if there is a servo attached.
    detach()    - Stops an attached servos from pulsing its i/o pin.
 */

#ifndef ServoPCA9685_h
#define ServoPCA9685_h

#include <inttypes.h>

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define INVALID_SERVO         255     // flag indicating an invalid servo index
#define SERVO_FREQ			       50	    // Hz

class Adafruit_PWMServoDriver;

class Servo
{
public:		// compatible methods for Servo Library
  Servo();
  uint8_t attach(const uint8_t pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(const uint8_t pin, const uint16_t min, const uint16_t max); // as above but also sets min and max values for writes.
  void detach();
  bool write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void writeMicroseconds(int value); // Write pulse width in microseconds
  uint8_t read();                        // returns current pulse width as an angle between 0 and 180 degrees
  uint16_t readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                   // return true if this servo is attached, otherwise false

public:		// additional methods
  void limit(const uint8_t low, const uint8_t high);	// limit servo movement to specified range of degrees

private:	// private methods
  static Adafruit_PWMServoDriver *initServoDriver();

private:
  static Adafruit_PWMServoDriver *servoDriver;

  uint8_t	pin;					 // channel nummer of servo on PCA9586 board
  uint8_t	lowerLimit;				 // minimum rotation degree limit
  uint8_t	upperLimit;			     // maximum rotation degree limit
  uint16_t	minPulse;                // minimum pulse width
  uint16_t	maxPulse;                // maximum pulse width
};

#endif /* ServoPCA9685_h */
