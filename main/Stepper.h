#ifndef Stepper_h
#define Stepper_h


class Stepper {
  public:
    Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2);
    Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4);
    Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4, int motor_pin_5);

    void setSpeed(long whatSpeed);
    void step(int number_of_steps);

  private:
    void stepMotor(int this_step);

    int direction;
    unsigned long step_delay;
    int number_of_steps;
    int pin_count;
    int step_number;


    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;
    int motor_pin_5;

    unsigned long last_step_time;
};

#endif
