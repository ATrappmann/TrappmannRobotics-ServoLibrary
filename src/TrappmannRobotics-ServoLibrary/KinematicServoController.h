/*
 * NAME: KinematicServoController.h
 */
#ifndef KINEMATIC_SERVO_CONTROLLER_H
#define KINEMATIC_SERVO_CONTROLLER_H

enum MoveMode {
  ACCELERATE,   // accelerate movement so that the destination is reached in the given time: a = 2s / t^2, v0 = 0
  CONSTANT,     // move with constant speed: v = s / t, a = 0
  DECELERATE,   // decelerate movement so that the destination is reached in the given time: a = -2s / t^2, v0 = s / t
  SMOOTH10,     // during movement: 10% acceleration, 80% constant speed, 10% deceleration
  SMOOTH20,     // during movement: 20% acceleration, 60% constant speed, 20% deceleration
  SMOOTH30      // during movement: 30% acceleration, 40% constant speed, 30% deceleration
};

/*
 * Single move for a servo to a destination position.
 * The speed is calculated by the given duration and the type
 * of movement is specified by mode.
 */
struct ServoMove {
  uint8_t   servoNo;        // number of servo in servoList
  uint8_t   endPosition;    // 0 - 180 degrees
  uint16_t  duration;       // in ms
  MoveMode  mode;           // type of movement
};

/*
 * Group of ServoMoves, starting at the same time.
 * The startTime is relative to the start of the Gesture.
 */
struct ServoGroup {
  ServoMove *moves;
  uint8_t    numMoves;
  uint32_t   startTime;      // common start time in ms
};

/*
 * ServoGesture defines a complex movement of numerous servos
 * starting at different times and moving with different speeds.
 */
struct ServoGesture {
  ServoGroup *groups;
  uint8_t     numGroups;
};

class KinematicServo;

class KinematicServoController {
private:
  KinematicServo *servoList;
  uint8_t         numServos;
  ServoGesture   *gestureList;
  uint8_t         numGestures;
  uint8_t         updateInterval; // in ms

  ServoGesture   *currentGesture;
  uint32_t        startMillis;
  uint32_t        lastMillis;

public:
  KinematicServoController(const KinematicServo *servos, const uint8_t numServos,
                           const ServoGesture *gestures, const uint8_t numGestures,
                           const uint8_t updateInterval = 5);

   /*
    * inititialize controller, attach servo and move to initial position
    * @return false, if one move has to be faster than the max. possible servo speed.
    */
   bool init(ServoGroup *initialGroup = NULL);

   /*
    * start timers
    * @return false, if illegal gestureNo was given.
    */
   bool begin(const uint8_t gestureNo);

   /*
    * regulary called in loop() to update servos postions
    */
   void update();

   /*
    * check if running gesture is done
    */
   bool isDone() const;

   /*
    * end gesture and detach all servos
    */
   void end();

private:
   /*
    * get timer and scale time to slow motion if necessary
    */
   uint32_t getTime() const;

};

#endif /* KINEMATIC_SERVO_CONTROLLER_H */
