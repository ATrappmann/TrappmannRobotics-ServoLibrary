# TrappmannRobotics-ServoLibrary

The **TrappmannRobotics-ServoLibrary** defines a **GenericServo** interface which is given by the original
[Servo Library for Arduino](https://github.com/arduino-libraries/Servo) and implements different types
of Servo actuation. The class **ServoPWM** is fully compatible to the original **Servo** class but has
some additional functions. The class **ServoPCA9685** implements a **Servo** which is driven by the 
[Adafruit-PWM-Servo-Driver-library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
which allows us to control 16 Servos via an I2C-bus controlled PCA9685-board
as it is sold from Adafruit [here](https://www.adafruit.com/product/815).

## Copyright
**TrappmannRobotics-ServoLibrary** is written by Andreas Trappmann from
[Trappmann-Robotics.de](https://www.trappmann-robotics.de/). It
is published under the MIT license, check LICENSE for more information.
All text above must be included in any redistribution.

## Release Notes

Version 1.0 - 01.03.2021

	* Initial version
