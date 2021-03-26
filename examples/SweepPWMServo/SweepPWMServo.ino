// NAME: SweepPWMServo.ino
//
#include <TrappmannRobotics-ServoLibrary/ServoPWM.h>"

ServoPWM  myservo;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // wait for Leonardo

  myservo.attach(9);

  Serial.print("MIN_PULSE_WIDTH: "); Serial.println(MIN_PULSE_WIDTH);
  myservo.writeMicroseconds(MIN_PULSE_WIDTH);
  delay(5000);

  Serial.print("DEFAULT_PULSE_WIDTH: "); Serial.println(DEFAULT_PULSE_WIDTH);
  myservo.writeMicroseconds(DEFAULT_PULSE_WIDTH);
  delay(5000);

  Serial.print("MAX_PULSE_WIDTH: "); Serial.println(MAX_PULSE_WIDTH);
  myservo.writeMicroseconds(MAX_PULSE_WIDTH);
  delay(5000);
}

void loop() {
  for (int pos=0; pos<=180; pos++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (int pos=MAX_PULSE_WIDTH; pos>=MIN_PULSE_WIDTH; pos--) { // goes from 180 degrees to 0 degrees
    myservo.writeMicroseconds(pos);  // tell servo to go to position in variable 'pos'
  }
}
