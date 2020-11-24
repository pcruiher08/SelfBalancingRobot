#include "Arduino.h"
#include "Stepper.h"

Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2){
  this->step_number = 0;
  this->direction = 0;
  this->last_step_time = 0;
  this->number_of_steps = number_of_steps;

  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;

  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);

  this->motor_pin_3 = 0;
  this->motor_pin_4 = 0;
  this->motor_pin_5 = 0;

  this->pin_count = 2;
}

Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4){
  this->step_number = 0;
  this->direction = 0; 
  this->last_step_time = 0;
  this->number_of_steps = number_of_steps;

  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;

  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);
  pinMode(this->motor_pin_3, OUTPUT);
  pinMode(this->motor_pin_4, OUTPUT);

  this->motor_pin_5 = 0;
  this->pin_count = 4;
}

Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4, int motor_pin_5){
  this->step_number = 0;
  this->direction = 0;
  this->last_step_time = 0;
  this->number_of_steps = number_of_steps;


  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;
  this->motor_pin_5 = motor_pin_5;


  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);
  pinMode(this->motor_pin_3, OUTPUT);
  pinMode(this->motor_pin_4, OUTPUT);
  pinMode(this->motor_pin_5, OUTPUT);

  this->pin_count = 5;
}

void Stepper::setSpeed(long whatSpeed){
  this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
}

void Stepper::step(int steps_to_move){
  int steps_left = abs(steps_to_move);
  if (steps_to_move > 0) { this->direction = 1; }
  if (steps_to_move < 0) { this->direction = 0; }

  while (steps_left > 0){
    unsigned long now = micros();
    if (now - this->last_step_time >= this->step_delay){
      this->last_step_time = now;
      if (this->direction == 1){
        this->step_number++;
        if (this->step_number == this->number_of_steps) {
          this->step_number = 0;
        }
      }
      else{
        if (this->step_number == 0) {
          this->step_number = this->number_of_steps;
        }
        this->step_number--;
      }
      steps_left--;
      if (this->pin_count == 5)
        stepMotor(this->step_number % 10);
      else
        stepMotor(this->step_number % 4);
    }
  }
}

void Stepper::stepMotor(int thisStep){
  if (this->pin_count == 2) {
    switch (thisStep) {
      case 0:  // 01
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
      break;
      case 1:  // 11
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, HIGH);
      break;
      case 2:  // 10
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
      break;
      case 3:  //00
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
      break;
    }
  }
  if (this->pin_count == 4) {
    switch (thisStep) {
      case 0:  // 1010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 1:  // 0110
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 2:  //0101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
      case 3:  //1001
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
    }
  }

  if (this->pin_count == 5) {
    switch (thisStep) {
      case 0:  // 01101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 1:  // 01001
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 2:  // 01011
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 3:  // 01010
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 4:  // 11010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 5:  // 10010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 6:  // 10110
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 7:  // 10100
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 8:  // 10101
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 9:  // 00101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
    }
  }
}
