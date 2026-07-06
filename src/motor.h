#pragma once
struct Motor {
    int pin1;
    int pin2;
    bool isOn = false;
    bool isForward = true;
    float distance = 0.0f;
    float rotations = 0.0f;
    Motor(int input_pin1, int input_pin2);
   //negative speed = backwards 
   void drive(float speed);

};
