/*
 * NAME: GenericServo.h
 *
 * DESC: Abstract base class for managing servos with different technical
 *       intrerfaces. The interface is basically compatible to the original
 *       Servo library from Arduino but does some cleanup and also adds
 *       advanced features.
 *
 * TrappmannRobotics developed the following Servo implementations so far:
 *    - ServoPWM, for Servos controlled by a PWM pin on an Arduino.
 *    - ServoPCA9685, for Servos controlled via a PWM channel on an I2C-bus controlled PCA9685-board.
 *
 * SOURCE: Code is available at https://github.com/ATrappmann/ServoPCA9685
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
#ifndef GENERIC_SERVO_H
#define GENERIC_SERVO_H

#include <inttypes.h>

#define INVALID_SERVO         255     // flag indicating an invalid servo index

class GenericServo {
protected:
  uint8_t pin;

public:
  GenericServo() { this->pin = INVALID_SERVO; }
  GenericServo(const uint8_t pin) { this->pin = pin; }
  uint8_t attach() { attach(pin); }

  /*
   * attach the given pin/channel for controlling the servo and power it up.
   * @param pin: pin or channel number the servo is controlled with.
   * @return servoId on success, INVALID_SERVO (255) in case of error.
   */
  virtual uint8_t attach(const uint8_t pin) = 0;

  /*
   * as above but also sets min and max values for the PWM pulse width in microseconds.
   * @param pin: pin or channel number the servo is controlled with.
   * @param minPulse: the pulse width, in microseconds, corresponding to the minimum (0-degree) angle on the servo (defaults to 544)
   * @param maxPulse: the pulse width, in microseconds, corresponding to the maximum (180-degree) angle on the servo (defaults to 2400)
   * @return servoId on success, INVALID_SERVO (255) in case of error
   */
  virtual uint8_t attach(const uint8_t pin, const uint16_t minPulse, const uint16_t maxPulse) = 0;

  /*
   * detach the servo from its pin/cannel and release power from the servo.
   */
  virtual void detach() = 0;

  /*
   * write a value to the servo, controlling the shaft accordingly. On a standard servo,
   * this will set the angle of the shaft (in degrees), moving the shaft to that orientation.
   * On a continuous rotation servo, this will set the speed of the servo (with 0 being
   * full-speed in one direction, 180 being full speed in the other, and a value near 90 being
   * no movement).
   * If the limitter is set, positions are clipped to the lower and upper limit.
   * @param position is treated as an angle in degree (0-180)
   * INCOMPATIBILTY NOTE: position > MIN_PULSE_WIDTH will not lead a direct call to writeMicroseconds
   */
  virtual void write(uint8_t position) = 0;

  /*
   * writeAnalog is an additional write method, nonexistent in the original Servo library,
   * which allows writing 10-bit values directly from an Arduino ADC to the servo.
   * @param adcValue is a 10-bit value from an Arduino ADC (range: 0-1023)
   */
  virtual void writeAnalog(uint16_t adcValue) = 0;

  /*
   * writeMicroseconds - Writes a value in microseconds (uS) to the servo, controlling
   * the shaft accordingly. On a standard servo, this will set the angle of the shaft.
   * On standard servos a parameter value of 1000 is fully counter-clockwise, 2000 is
   * fully clockwise, and 1500 is in the middle.
   * @param pulse - the value of the pulse with in microseconds
   */
  virtual void writeMicroseconds(uint16_t pulse) = 0; // Write pulse width in microseconds

  /*
   * read the current angle of the servo.
   * @return current angle between 0 and 180 degrees
   */
  virtual uint8_t read() = 0;

  /*
   * readMicroseconds of current pulse width of the servo.
   * @return current pulse width in microseconds for this servo
   */
  virtual uint16_t readMicroseconds() = 0;

  /*
   * attached - Check whether the Servo is attached to a pin.
   * @return true if this servo is attached, otherwise false
   */
  virtual bool attached() const = 0;

  /*
   * limit - set Servo limitter and allow movement only in the specified range of degrees
   * during calls to write().
   * @param lowerLimit - set the lower limit of the moving range
   * @param upperLimit - set the upper limit of the moving range
   */
  virtual void limit(uint8_t lowerLimit, uint8_t upperLimit) = 0;
  virtual uint8_t getLowerLimit() const = 0;
  virtual uint8_t getUpperLimit() const = 0;
};

#endif /* GENERIC_SERVO_H */
