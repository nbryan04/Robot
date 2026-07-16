#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <ESP32Encoder.h>
#include "config.h"
#include <functional>

#pragma once
struct Motor {
    int forwardPin;
    int reversePin;
    int encoderPin1;
    int encoderPin2;
    std::function<int(float)> mapSpeedToDutyCycle;

    int circumference = PI * robotConfig::WHEEL_1_DIAMETER;

    int lastSpeedTime = 0;
    int lastEncoderCount = 0;
    double currentSpeed = 0.0;
    
    enum MovementState { Idle, Driving, Braking };
    MovementState movementState = Idle;
    long targetEncoderCount = 0;
    long brakeTargetCount = 0;
    int currentDriveDirection = robotConfig::STOPPED;
    void update();

    enum MotorState { Forward, Reverse, Stopped };
    ESP32Encoder encoder;
    Motor(int input_pin1, int input_pin2, int inputEncoderPin1, int inputEncoderPin2, float diameter, int polarity,
          std::function<int(float)> speedToDutyCallback);
    // negative speed = backwards
    void drive(int dutyCycle, int direction);
    void driveDistance(float distance, float speed = 1.0f);
    void oneTurn(void);
    void begin();
    double speed();
    void enableQuadratureEncoder();
    MotorState motorState;
};
