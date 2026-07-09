#include <Arduino.h>
#include <CircularBuffer.hpp>
#pragma once
struct Motor {
    int forwardPin;
    int backwardsPin;
    int encoderPin1;
    int encoderPin2;
    int encoderCount;
    int sampleCount;
    float wheelDiameter;
    bool isOn = false;
    bool isForward = true;
    float distance = 0.0f;
    float rotations = 0.0f;
    bool encoderEnabled = false;
    CircularBuffer<int, 10> sampleBuffer;
    Motor(int input_pin1, int input_pin2, int inputEncoderPin1, int inputEncoderPin2, float diameter, int polarity = 1);
   //negative speed = backwards 
   void drive(float speed, int direction);
   void driveDistance(float distance, float speed = 1.0f);
   void one_turn(void);
   void begin();
   float speed(int n);

   void increaseCount(int num = 1);
   void resetCount();
   void enableEncoder();
   void disableEncoder();
   void handleInterrupt();
};

