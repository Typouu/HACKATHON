#include <Arduino.h>
#include "motorControl.h"

Motor::Motor() {
  pinMode(LEFT1, OUTPUT);
  pinMode(LEFT2, OUTPUT);
  pinMode(RIGHT1, OUTPUT);
  pinMode(RIGHT2, OUTPUT);
  pinMode(LEFT_SPEED, OUTPUT);
  pinMode(RIGHT_SPEED, OUTPUT);
  this->leftSpeedVal = 0;
  this->rightSpeedVal = 0;
  this->state = STOP_STATE;
  stopCar();
}

int Motor::getSpeed(bool isRightMotor) {
  return isRightMotor ? this->rightSpeedVal : this->leftSpeedVal;
}

bool Motor::setState(int newState) { //return true is the newState is not the old state
  if (this->state != newState) {
    this->state = newState;
    return true;
  }
  return false;
}

void Motor::setSpeedTo(int newSpeed, bool isRightMotor) {
  newSpeed = min(newSpeed, MAX_SPEED);
  newSpeed = max(newSpeed, MIN_SPEED);
  if (isRightMotor) {
    analogWrite(RIGHT_SPEED, newSpeed);
    this->rightSpeedVal = newSpeed;
  } else {
    analogWrite(LEFT_SPEED, newSpeed);
    this->leftSpeedVal = newSpeed;
  }
}

void Motor::incrementSpeed(int increment, bool isRightMotor) {
  if (isRightMotor) {
    return setSpeedTo(rightSpeedVal + increment, isRightMotor);
  } else {
    return setSpeedTo(leftSpeedVal + increment, isRightMotor);
  }
}

void Motor::stopCar() {
  if (this->state != STOP_STATE) {
    this->setState(STOP_STATE);
    digitalWrite(RIGHT1, LOW);
    digitalWrite(RIGHT2, LOW);
    digitalWrite(LEFT1, LOW);
    digitalWrite(LEFT2, LOW);
    analogWrite(RIGHT_SPEED, 0);
    analogWrite(LEFT_SPEED, 0);
  }
}

void Motor::forward() { //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  if (this->setState(FORWARD_STATE)) {
    digitalWrite(RIGHT1, HIGH); //the right motor rotates FORWARDS when RIGHT1 is HIGH and RIGHT2 is LOW
    digitalWrite(RIGHT2, LOW);
    digitalWrite(LEFT1, HIGH);
    digitalWrite(LEFT2, LOW);
  }
}

void Motor::left() { //rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  if (this->setState(LEFT_STATE)) {
    digitalWrite(RIGHT1, HIGH);
    digitalWrite(RIGHT2, LOW);
    digitalWrite(LEFT1, LOW);
    digitalWrite(LEFT2, HIGH);
  }
}

void Motor::right() {
  if (this->setState(RIGHT_STATE)) {
    digitalWrite(RIGHT1, LOW);
    digitalWrite(RIGHT2, HIGH);
    digitalWrite(LEFT1, HIGH);
    digitalWrite(LEFT2, LOW);
  }
}

// debugging prints
void Motor::printInfo() {
  print_class_name("motor");
  PRINT_VAR("leftSpeedVal", this->leftSpeedVal);
  PRINT_VAR("rightSpeedVal", this->rightSpeedVal);
  PRINT_VAR("state", this->state_string(this->state));
}

String Motor::state_string(int state) {
  return state_string_helper(state) + " (" + String(state) + ")";
}

String Motor::state_string_helper(int state) {
  switch (state) {
    case LEFT_STATE:
    return "LEFT_STATE";
    case RIGHT_STATE:
    return "RIGHT_STATE";
    case FORWARD_STATE:
    return "FORWARD_STATE";
    case STOP_STATE:
    return "STOP_STATE";
  }
  return "unknown";
}
