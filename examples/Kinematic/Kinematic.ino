/*
 * NAME: Kinematic.ino
 */

#include <Arduino.h>
#include <TrappmannRobotics.h>
#include <TrappmannRobotics-ServoLibrary.h>

#define DEBUG 1
#include <TrappmannRobotics/Debug.h>

#define SERVO1_PIN  9
#define SERVO2_PIN  10

enum ServoNo { SERVO1, SERVO2 };
KinematicServo servos[] = {
  KinematicServo(new ServoPWM(SERVO1_PIN), 90, 0, 180),
  KinematicServo(new ServoPWM(SERVO2_PIN), 90, 0, 180)
};

ServoMove sweep1[] = {
  { SERVO1, 180, 100, CONSTANT },   // servo1 to max
  { SERVO2, 0,   100, ACCELERATE }  // servo2 to min
};
ServoMove sweep2[] = {
  { SERVO1, 0,   100, CONSTANT },   // servo1 to min
  { SERVO2, 180, 100, ACCELERATE }  // servo2 to max
};
ServoMove toMaxMove[] = {
  { SERVO1, 180, 100, CONSTANT },   // servo1 to max
  { SERVO2, 180, 100, CONSTANT }    // servo2 to max
};

ServoGroup sweep[] = {
  { sweep1, sizeof(sweep1)/sizeof(ServoMove), 0   },  // start at the beginning of gesture
  { sweep2, sizeof(sweep2)/sizeof(ServoMove), 600 }   // start 600ms after start of gesture, 500ms after end of sweep1
};
ServoGroup toMax[] = {
  { toMaxMove, sizeof(toMaxMove)/sizeof(ServoMove), 0 }
};

ServoGesture gestures[] = {
  { sweep, sizeof(sweep)/sizeof(ServoGroup) },
  { toMax, sizeof(toMax)/sizeof(ServoGroup) }
};

KinematicServoController controller(servos, sizeof(servos)/sizeof(KinematicServo),
                                    gestures, sizeof(gestures)/sizeof(ServoGesture));

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial << F("------------------------------\n");
  Serial << F("Sketch: ") << getBaseName(__FILE__) << LF;
  Serial << F("Running...\n");

  if (!controller.init()) {
    Serial << F("FATAL ERROR in configuration of servo moves!\nExiting...\n");
    Serial.flush();
    exit(-1);
  }

  controller.begin(0);
}

int cnt = 0;
void loop() {
  controller.update();
  if (controller.isDone()) {
    if (++cnt < 10) {
      controller.begin(0);  // restart sweep gesture
    }
    else if (cnt < 11) {
      controller.begin(1);  // toMax gesture
    }
    else {
      controller.end();
      Serial << F("Done. - Press RESET to restart\n");
      Serial.flush();
      exit(0);
    }
  }
}
