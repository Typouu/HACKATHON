#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "commonlib.h"

#define MAX_SPEED 255 //max PWM value written to motor speed pin. It is typically 255.
#define MIN_SPEED 35 //min PWM value written to motor speed pin
// #define MIN_MOVING_SPEED 105 //min PWM value at which motor moves
#define EQUILIBRIUM_SPEED 248
#define LEFT false
#define RIGHT true

//control pins for left and right motors
#define LEFT_SPEED 5 //means pin 9 on the Arduino controls the speed of left motor
#define RIGHT_SPEED 6

#define LEFT1 2 //left 1 and left 2 control the direction of rotation of left motor
#define LEFT2 3
#define RIGHT1 4
#define RIGHT2 8

//states
#define LEFT_STATE 0
#define RIGHT_STATE 1
#define FORWARD_STATE 2
#define STOP_STATE 3

#define PROLONGED_TIME_THRESHOLD 2000 //in ms

//motorControl.cpp functions
class Motor {
  private:
    int leftSpeedVal;
    int rightSpeedVal;
    int state;
    String Motor::state_string(int state);
    String Motor::state_string_helper(int state);
    bool Motor::setState(int newState);

  public:
    Motor::Motor();
    void Motor::setSpeedTo(int newSpeed, bool isRightMotor);
    void Motor::incrementSpeed(int increment, bool isRightMotor);
    void Motor::stopCar();
    void Motor::forward();
    void Motor::left();
    void Motor::right();
    int Motor::getSpeed(bool isRightMotor);
    void Motor::printInfo();
};

#endif
