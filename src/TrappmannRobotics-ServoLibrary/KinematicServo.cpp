/*
 * NAME: KinematicServo.cpp
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

#include <Arduino.h>
#include <TrappmannRobotics-ServoLibrary/KinematicServo.h>

KinematicServo::KinematicServo(const GenericServo *servo, const uint8_t initialPosition, const uint8_t minPos, const uint8_t maxPos, const uint16_t maxSpeed) {
   this->servo = servo;
   this->initialPosition = initialPosition;
   this->servo->limit(minPos, maxPos);
   this->maxSpeed = maxSpeed;

   currentPosition = initialPosition;
   currentSpeed = 0.0;
   currentAcceleration = 0.0;

   currentMove = NULL;
   gestureLength = 0.0;
   gestureDuration = 0.0;
}

uint8_t KinematicServo::attach() {
  return servo->attach();
}

uint8_t KinematicServo::attach(const uint8_t pin) {
  return servo->attach(pin);
}

uint8_t KinematicServo::attach(const uint8_t pin, const uint16_t min, const uint16_t max){
  return servo->attach(pin, min, max);
}

void KinematicServo::detach() {
  servo->detach();
}

void KinematicServo::write(uint8_t value) {
  servo->write(value);
}

void KinematicServo::writeAnalog(uint16_t value) {
  servo->writeAnalog(value);
}

void KinematicServo::writeMicroseconds(uint16_t value) {
  servo->writeMicroseconds(value);
}

uint8_t KinematicServo::read() {
  return servo->read();
}

uint16_t KinematicServo::readMicroseconds() {
  return servo->readMicroseconds();
}

bool KinematicServo::attached() const {
  return servo->attached();
}

void KinematicServo::limit(uint8_t low, uint8_t high) {
  servo->limit(low, high);
}

uint8_t KinematicServo::getLowerLimit() const {
  return servo->getLowerLimit();
}

uint8_t KinematicServo::getUpperLimit() const {
  return servo->getUpperLimit();
}

/*
 * KinematicServo interface
 */

void KinematicServo::setCurrentPosition(uint8_t pos) {
  pos = constrain(pos, servo->getLowerLimit(), servo->getUpperLimit());
  this->currentPosition = pos;
}

void KinematicServo::setCurrentSpeed(float speed) {
  if (speed > maxSpeed) speed = maxSpeed;
  this->currentSpeed = speed;
}

void KinematicServo::setCurrentAcceleration(const float accel) {
  this->currentAcceleration = accel;
}

void KinematicServo::setCurrentMove(ServoMove *move) {
  this->currentMove = move;
}

void KinematicServo::setGestureLength(const float degrees) {
  this->gestureLength = degrees;
}

void KinematicServo::setGestureDuration(const uint32_t duration) {
  this->gestureDuration = duration;
}
