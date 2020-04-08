/* Example sketch to control a stepper motor with TB6600 stepper motor driver, AccelStepper library and Arduino: acceleration and deceleration. More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 7
#define stepPin 6
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(500);
  stepper.setSpeed(500);
  stepper.setAcceleration(200);
}

void loop() {
  // Set the target position:
  delay(10000);
  stepper.setSpeed(100);
  stepper.moveTo(80000);
  // Run to target position with set speed and acceleration/deceleration:
  stepper.runToPosition();
  delay(60000);

  stepper.setSpeed(500);
  stepper.moveTo(0);
  // Run to target position with set speed and acceleration/deceleration:
  stepper.runToPosition();
  delay(60000);
}
