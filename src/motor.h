#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <ESP32Encoder.h>

#pragma once
struct Motor {
    int forwardPin;
    int reversePin;
    int encoderPin1;
    int encoderPin2;

    int circumference = PI * robotConfig::WHEEL_1_DIAMETER;

    int lastSpeedTime = 0;
    int lastEncoderCount = 0;
    double currentSpeed = 0.0;

    enum MotorState { Forward, Reverse, Stopped };
    ESP32Encoder encoder;
    Motor(int input_pin1, int input_pin2, int inputEncoderPin1, int inputEncoderPin2, float diameter, int polarity);
    // negative speed = backwards
    void drive(int dutyCycle, int direction);
    void driveDistance(float distance, float speed = 1.0f);
    void oneTurn(void);
    void begin();
    double speed();
    void enableQuadratureEncoder();
    MotorState motorState;
};
