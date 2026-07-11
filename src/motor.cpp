#include <Arduino.h>

#include <CircularBuffer.hpp>

#include "config.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "motor.h"
// TODO:
// PWM control
// Proper implementation of counter (interrupt)
//

Motor::Motor(int p1, int p2, int ePin1, float diameter,
             int polarity) {
    if (polarity == -1) {  // swap the pins
        forwardPin = p2;
        reversePin = p1;
    }

    else {
        forwardPin = p1;
        reversePin = p2;
    }
    encoderPin1 = ePin1;
    encoderCount = 0;
    sampleCount = 0;
    wheelDiameter = diameter;
    motorState = Stopped;
}

void Motor::begin() {
    ledcAttach(forwardPin, robotConfig::DRIVING_FREQUENCY,
               robotConfig::PWM_RESOLUTION);
    ledcAttach(reversePin, robotConfig::DRIVING_FREQUENCY,
               robotConfig::PWM_RESOLUTION);
    ledcWrite(forwardPin, 0);
    ledcWrite(reversePin, 0);
    enableEncoder();
}
void Motor::enableEncoder() {
    attachInterruptArg(
        digitalPinToInterrupt(encoderPin1),
        [](void* arg) IRAM_ATTR {
            Motor* motor = static_cast<Motor*>(arg);
            motor->handleInterrupt();
        },
        this, RISING);
    encoderEnabled = true;
}

void Motor::disableEncoder() {
    detachInterrupt(digitalPinToInterrupt(encoderPin1));
    encoderEnabled = false;
}
// TODO
// implement this so it can be called WHILE the motor is still moving.
void Motor::drive(int dutyCycle, int direction) {
    switch (direction) {
        case (robotConfig::FORWARD):
            if (motorState == Reverse) {
                ledcWrite(reversePin, 0);
                delay(robotConfig::PWM_DELAY);
            }
            ledcWrite(forwardPin, dutyCycle);
            motorState = Forward;
            break;

        case (robotConfig::REVERSE):
            if (motorState == Forward) {
                ledcWrite(forwardPin, 0);
                delay(robotConfig::PWM_DELAY);
            }
            ledcWrite(reversePin, dutyCycle);
            motorState = Reverse;
            break;

        case (robotConfig::STOPPED):
            ledcWrite(forwardPin, 0);
            ledcWrite(reversePin, 0);
            motorState = Stopped;
            break;

        default:
            ledcWrite(forwardPin, 0);
            ledcWrite(reversePin, 0);
            motorState = Stopped;
            break;
    }
}

void Motor::driveDistance(float distance, float speed) {}

/* Returns a float value corresponding to the average speed of the motor
 * \param n The number of samples in the buffer to use in the speed calculation
 * (n<10, n>0)
 * \return -1.0 if the sample buffer is empty, or if n >10, n<0 the average
 * speed otherwise.
 *
 *
 */

float Motor::speed(int n) {
    if (sampleBuffer.isEmpty() || n > 10 || n < 0 || sampleBuffer.size() < n) {
        return -1.0f;
    }

    else {
        return (PI * wheelDiameter / robotConfig::PULSES_REV) *
               (n * robotConfig::DOWNSAMPLING_FACTOR) /
               static_cast<float>(sampleBuffer.first() - sampleBuffer[n - 1]);
    }
}
void Motor::increaseCount() { encoderCount += 1; }
void Motor::resetCount() { encoderCount = 0; }
void IRAM_ATTR Motor::handleInterrupt() {
    if (sampleCount < robotConfig::DOWNSAMPLING_FACTOR) {
        sampleCount++;
    } else {
        sampleBuffer.unshift(millis());
        sampleCount = 0;
    }
    increaseCount();
}
