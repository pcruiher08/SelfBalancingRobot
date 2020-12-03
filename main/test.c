/*Example sketch to control a stepper motor with A4988 stepper motor driver, AccelStepper library and Arduino: continuous rotation. More info: https://www.makerguides.com */
// Include the AccelStepper library:
#include <AccelStepper.h>
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin1 19
#define stepPin1 18
#define dirPin2 4
#define stepPin2 2
#define motorInterfaceType 1// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

void setup() {
  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);

}
void loop() {
  // Set the speed in steps per second:
  stepper1.setSpeed(400);
  stepper2.setSpeed(400);

  // Step the motor with a constant speed as set by setSpeed():
  stepper1.runSpeed();
  stepper2.runSpeed();

}