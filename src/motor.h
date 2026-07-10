#include <Arduino.h>
#include <CircularBuffer.hpp>
#pragma once
struct Motor {
    int forwardPin;
    int reversePin;
    int encoderPin1;
    int encoderPin2;
    volatile int encoderCount;
    int sampleCount;
    float wheelDiameter;
    bool isOn = false;
    bool isForward = true;
    float distance = 0.0f;
    float rotations = 0.0f;
    bool encoderEnabled = false;
    enum MotorState {
       Forward,
       Reverse,
       Stopped
   };
    CircularBuffer<int, 10> sampleBuffer;
    Motor(int input_pin1, int input_pin2, int inputEncoderPin1, int inputEncoderPin2, float diameter, int polarity = 1);
   //negative speed = backwards 
   void drive(int dutyCycle, Motor::MotorState direction);
   void driveDistance(float distance, float speed = 1.0f);
   void one_turn(void);
   void begin();
   float speed(int n);

   void increaseCount();
   void resetCount();
   void enableEncoder();
   void disableEncoder();
   void handleInterrupt();
   MotorState motorState;
};

