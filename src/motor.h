#include <Arduino.h>
#pragma once
struct Motor {
    int forwardPin;
    int backwardsPin;
    int encoderPin1;
    int encoderPin2;
    int pulseCount;
    bool isOn = false;
    bool isForward = true;
    float distance = 0.0f;
    float rotations = 0.0f;
    Motor(int input_pin1, int input_pin2, int inputEncoderPin1, int inputEncoderPin2, int polarity);
   //negative speed = backwards 
   void drive(float speed, int direction);
   void drive_distance(float distance, float speed = 1.0f);
   void one_turn(void);
   void begin();
   float speed();
   void increaseCount();
   void resetCount();
};

void IRAM_ATTR motor1ISR();
void IRAM_ATTR motor2ISR();
