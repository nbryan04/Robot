#pragma once
#include "motor.h"
struct drive_train {
  drive_train(const Motor &motor1, const Motor &motor2);

  void turn(float angle); // int method?
  void drive(float distance, float speed); //set a default speed
  void line_follow(float distance, float speed);
}
