/*
 * NAME: KinematicServo.h
 *
 * DESC: This class adds kinematic features for the movement of a servo.
 *
 * This file is part of the TrappmannRobotics-ServoLibrary.
 *
 * SOURCE: Code is available at https://github.com/ATrappmann/TrappmannRobotics-ServoLibrary
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
#ifndef KINEMATIC_SERVO_H
#define KINEMATIC_SERVO_H

#include <TrappmannRobotics-ServoLibrary/GenericServo.h>

struct ServoMove;

class KinematicServo: public GenericServo {
private:
  GenericServo *servo;
  uint16_t  maxSpeed;   // in °/s

  uint8_t initialPosition;
  float   currentPosition;
  float   currentSpeed;
  float   currentAcceleration;

  ServoMove *currentMove;
  float     gestureLength;    // in +/- degrees
  uint32_t  gestureDuration;  // in ms

public:
  KinematicServo(const GenericServo *servo, const uint8_t initalPosition, const uint8_t minPos, const uint8_t maxPos, const uint16_t maxSpeed = 600);
  uint8_t attach();
  
public: // GenericServo interface
  uint8_t attach(const uint8_t pin);      // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(const uint8_t pin, const uint16_t min, const uint16_t max); // as above but also sets min and max values for writes.
  void detach();
  void write(uint8_t value);              // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void writeAnalog(uint16_t value); // write corresponding pulse for value from ADC (range: 0-1023)
  void writeMicroseconds(uint16_t value); // Write pulse width in microseconds
  uint8_t read();                         // returns current pulse width as an angle between 0 and 180 degrees
  uint16_t readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached() const;                  // return true if this servo is attached, otherwise false
  void limit(uint8_t low, uint8_t high);  // limit servo movement to specified range of degrees
  uint8_t getLowerLimit() const;
  uint8_t getUpperLimit() const;

public: // KinematicServo interface
  uint16_t getMaxSpeed() const { return maxSpeed; }
  uint8_t  getInitialPosition() const { return initialPosition; }

  uint8_t getCurrentPosition() const { return currentPosition; }
  void    setCurrentPosition(uint8_t pos);

  float   getCurrentSpeed() const { return currentSpeed; }
  void    setCurrentSpeed(float speed);

  float   getCurrentAcceleration() const { return currentAcceleration; }
  void    setCurrentAcceleration(const float accel);

  ServoMove *getCurrentMove() const { return currentMove; }
  void       setCurrentMove(ServoMove *move);

  float   getGestureLength() const { return gestureLength; }
  void    setGestureLength(const float degrees);

  uint32_t  getGestureDuration() const { return gestureDuration; }
  void      setGestureDuration(const uint32_t duration);
};

#endif /* KINEMATIC_SERVO_H */
