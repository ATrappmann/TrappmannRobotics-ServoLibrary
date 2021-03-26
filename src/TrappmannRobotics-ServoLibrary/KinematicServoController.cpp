/*
 * NAME: KinematicServoController.cpp
 */
#include <Arduino.h>
#include <TrappmannRobotics.h>
#include <TrappmannRobotics-ServoLibrary/KinematicServoController.h>
#include <TrappmannRobotics-ServoLibrary/KinematicServo.h>

#define DEBUG 1
#include <TrappmannRobotics/Debug.h>

#define TIME_SCALE 10

KinematicServoController::KinematicServoController(const KinematicServo *servoList, const uint8_t numServos,
                         const ServoGesture *gestures, const uint8_t numGestures,
                         const uint8_t updateInterval) {
  this->servoList = servoList;
  this->numServos = numServos;
  this->gestureList = gestures;
  this->numGestures = numGestures;
  this->updateInterval = updateInterval;
}

/*
* inititialize controller, attach servo and move to initial position
* @return false, if one move has to be faster than the max. possible servo speed.
*/
bool KinematicServoController::init(ServoGroup *initialGroup = NULL) {
  SEROUT(F("KinematicServoController::init()\n"));
  for (uint8_t no=0; no<numServos; no++) {
    servoList[no].setCurrentPosition(servoList[no].getInitialPosition());
  }

  // check movements for their max speed
  for (uint8_t g=0; g<numGestures; g++) {
    ServoGesture *gesture = &gestureList[g];
    SEROUT(F("gesture[") << g << "] = $" << toHexString((uint16_t)gesture) << LF);

    for (uint8_t gr=0; gr<gesture->numGroups; gr++) {
      ServoGroup *group = &gesture->groups[gr];
      SEROUT(F("\tgroup[") << gr << "] = $" << toHexString((uint16_t)group) << LF);

      for (uint8_t m=0; m<group->numMoves; m++) {
        ServoMove *move = &group->moves[m];
        KinematicServo *servo = &servoList[move->servoNo];
        SEROUT(F("\t\tm[") << m << "] = $" << toHexString((uint16_t)move));

        float t = (move->duration) / 1000.0;  // in seconds
        float s = float(move->endPosition) - servo->getCurrentPosition();
        SEROUT( F(": s=") << move->servoNo << F(", pos=") << servo->getCurrentPosition() << F(", dest=") << move->endPosition << F(", s=") << s << F(", t=") << t);

        float accel, speed;
        switch (move->mode) {
          case ACCELERATE:
          case DECELERATE:
            accel = 2.0*s / (t*t);
            speed = accel * t;
            break;
          case CONSTANT:
            accel = 0.0;
            speed = s / t;
            break;
          case SMOOTH10:  // 10% beschleunigen, 80% kosntant, 10% bremsen
            accel = s / (0.09 * t*t);
            speed = accel * t;
            break;
          case SMOOTH20:  // 20% beschleunigen, 60% konstant, 20% bremsen
            accel = s / (0.16 * t*t);
            speed = accel * t;
            break;
          case SMOOTH30:  // 30% beschleunigen, 40% konstant, 30% bremsen
            accel = s / (0.21 * t*t);
            speed = accel * t;
            break;
          default:
            Serial << F("\nFATAL ERROR: Move #") << m << F(" has unknown mode ") << move->mode << LF;
            return false;
        }
        SEROUT(F(", accel=") << accel << F(", speed=") << speed << LF);
        if (abs(speed) >= servo->getMaxSpeed()) {
          Serial << F("ERROR: Move #") << m << F(" with speed of ") << abs(speed) << F("°/s is to fast for servo (maxSpeed=") << servo->getMaxSpeed() << "°/s)\n";
          return false;
        }
        servo->setCurrentPosition(move->endPosition);
      }
    }
  }

  for (uint8_t no=0; no<numServos; no++) {
    servoList[no].setCurrentPosition(servoList[no].getInitialPosition());
  }

  if (NULL != initialGroup) {
    for (int m=0; m<initialGroup->numMoves; m++) {
      ServoMove *move = &initialGroup->moves[m];
      servoList[move->servoNo].setCurrentPosition(move->endPosition);
    }
  }

  for (uint8_t no=0; no<numServos; no++) {
    servoList[no].write(servoList[no].getCurrentPosition());
    servoList[no].attach();
  }

  return true;
}

/*
* start timers
* @return false, if illegal gestureNo was given.
*/
bool KinematicServoController::begin(const uint8_t gestureNo) {
  SEROUT(F("KinematicServoController::begin(") << gestureNo << F(") "));
  if (gestureNo >= numGestures) {
    SEROUT(F("ERROR: Illegal gestureNo\n"));
    currentGesture = NULL;
    return false;
  }

  currentGesture = &gestureList[gestureNo];
  startMillis = getTime();
  lastMillis = 0L;
  SEROUT("startTime=" << startMillis << F(", numGroups=") << currentGesture->numGroups << LF);
  return true;
}

/*
* regulary called in loop() to update servoList postions
*/
void KinematicServoController::update() {
  if (NULL == currentGesture) return;

  uint32_t currentMillis = getTime() - startMillis;
  uint32_t delta = currentMillis - lastMillis;
  if (delta < updateInterval) return; // update only every 20ms
  lastMillis = currentMillis;

  float dt = delta / 1000.0; // convert to seconds

  bool updated = false;
  for (uint8_t g=0; g<currentGesture->numGroups; g++) {
    ServoGroup *group = &currentGesture->groups[g];
    if (currentMillis >= group->startTime) {
      for (uint8_t m=0; m<group->numMoves; m++) {
        ServoMove *move = &group->moves[m];
        if (currentMillis <= (group->startTime + move->duration)) {
          SEROUT("G" << g << "M" << m << " ");

          updated = true;
          KinematicServo *servo = &servoList[move->servoNo];
          if (move != servo->getCurrentMove()) {
            servo->setCurrentMove(move);
            servo->setGestureLength(float(move->endPosition) - servo->getCurrentPosition());
            servo->setGestureDuration(move->duration);

            float t = float(servo->getGestureDuration()) / 1000.0;
            switch (move->mode) {
              case ACCELERATE:
                servo->setCurrentAcceleration(2.0*servo->getGestureLength() / (t*t));
                servo->setCurrentSpeed(0.0);
                break;
              case CONSTANT:
                servo->setCurrentAcceleration(0.0);
                servo->setCurrentSpeed(servo->getGestureLength() / t);
                break;
              case DECELERATE:
                servo->setCurrentAcceleration(-2.0*servo->getGestureLength() / (t*t));
                servo->setCurrentSpeed(-servo->getCurrentAcceleration() * t);
                break;
              default:
                servo->setCurrentAcceleration(0.0);
                servo->setCurrentSpeed(0.0);
                break;
            }
          }

          if ((SMOOTH10 == move->mode) || (SMOOTH20 == move->mode) || (SMOOTH30 == move->mode)) {
            float t = float(servo->getGestureDuration()) / 1000.0;

            uint32_t smoothPeriod = 0.0;
            switch (move->mode) {
              case SMOOTH10:
                smoothPeriod = 0.1 * float(servo->getGestureDuration());
                break;
              case SMOOTH20:
                smoothPeriod = 0.2 * float(servo->getGestureDuration());
                break;
              case SMOOTH30:
                smoothPeriod = 0.3 * float(servo->getGestureDuration());
                break;
            }

            if (currentMillis <= (group->startTime + smoothPeriod)) {  // beschleunigen
              switch (move->mode) {
                case SMOOTH10:
                  servo->setCurrentAcceleration(servo->getGestureLength() / (0.09 * t*t));
                  break;
                case SMOOTH20:
                  servo->setCurrentAcceleration(servo->getGestureLength() / (0.16 * t*t));
                  break;
                case SMOOTH30:
                  servo->setCurrentAcceleration(servo->getGestureLength() / (0.21 * t*t));
                  break;
              }
            }
            else if (currentMillis <= (group->startTime + move->duration - smoothPeriod)) {  // konstant
              servo->setCurrentAcceleration(0.0);
            }
            else  { // bremsen
              switch (move->mode) {
                case SMOOTH10:
                  servo->setCurrentAcceleration(-servo->getGestureLength() / (0.09 * t*t));
                  break;
                case SMOOTH20:
                  servo->setCurrentAcceleration(-servo->getGestureLength() / (0.16 * t*t));
                  break;
                case SMOOTH30:
                  servo->setCurrentAcceleration(-servo->getGestureLength() / (0.21 * t*t));
                  break;
              }
            }
          }

          servo->setCurrentPosition(servo->getCurrentPosition() + (servo->getCurrentAcceleration()/2.0 * dt*dt) + (servo->getCurrentSpeed() * dt));
          servo->setCurrentSpeed(servo->getCurrentSpeed() + (servo->getCurrentAcceleration() * dt));

          uint8_t pos = round(servo->getCurrentPosition()); //(servo->getCurrentPosition() * 10.0 + 5.0) / 10.0;
          pos = constrain(pos, servo->getLowerLimit(), servo->getUpperLimit());
          servo->write(pos);
        }
      }
    }
  }

  if (updated) {
    SEROUT("T=" << currentMillis << ": ");
    for (uint8_t no=0; no<numServos; no++) {
      int16_t pos = (servoList[no].getCurrentPosition() * 10.0 + 5.0) / 10.0;
      if ((pos < 0) || (pos > 180))  SEROUT("*");
      SEROUT("S" << no << ": " << servoList[no].getCurrentPosition() << " " << servoList[no].getCurrentSpeed() << " " << servoList[no].getCurrentAcceleration() << " ");
    }
    SEROUT(LF);
  }
}

/*
* check if running gesture is done
*/
bool KinematicServoController::isDone() const {
  if (NULL == currentGesture) return true;
  uint32_t currentMillis = getTime() - startMillis;

  bool isDone = true;
  for (uint8_t g=0; g<currentGesture->numGroups; g++) {
    ServoGroup *group = &currentGesture->groups[g];
    for (uint8_t m=0; m<group->numMoves; m++) {
      ServoMove *move = &group->moves[m];
      if (currentMillis < group->startTime + move->duration) {
        return false;
      }
    }
  }

  for (uint8_t g=0; g<currentGesture->numGroups; g++) {
    ServoGroup *group = &currentGesture->groups[g];
    for (uint8_t m=0; m<group->numMoves; m++) {
      ServoMove *move = &group->moves[m];
      SEROUT("SF" << move->servoNo << "=" << servoList[move->servoNo].getCurrentPosition() << "/" << move->endPosition << LF);
      servoList[move->servoNo].write(move->endPosition);
      servoList[move->servoNo].setCurrentPosition(move->endPosition);
    }
  }

#ifdef DEBUG
  if (isDone) {
    Serial << F("Final Positions: ");
    for (uint8_t i=0; i<numServos; i++) {
      int8_t pos = servoList[i].getCurrentPosition() - servoList[i].getInitialPosition();
      Serial << "S" << i << "=" << pos << " ";
    }
    Serial << LF;
  }
#endif

  return isDone;
}

/*
* end gesture and detach all servos
*/
void KinematicServoController::end() {
  SEROUT(F("GestureController::end()\n"));
  for (uint8_t no=0; no<numServos; no++) {
    servoList[no].detach();
  }
  currentGesture = NULL;
}

/*
* get timer and scale time to slow motion if necessary
*/
uint32_t KinematicServoController::getTime() const {
  return millis() / TIME_SCALE;
}
