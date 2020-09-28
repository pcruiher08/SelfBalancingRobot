#include <Stepper.h>

#define PASOS 100

Stepper stepper(PASOS, 8, 9, 10, 11);

int previous = 0;

void setup(){
  stepper.setSpeed(30);
}

void loop() {
  int val = analogRead(0);
  stepper.step(val - previous);
  previous = val;
}