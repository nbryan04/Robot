#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <ESP32Encoder.h>

#include "config.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "motor.h"

Motor::Motor(int p1, int p2, int ePin1, int ePin2, float diameter, int polarity) {
    if (polarity == -1) {  // swap the pins
        forwardPin = p2;
        reversePin = p1;
    }

    else {
        forwardPin = p1;
        reversePin = p2;
    }
    encoderPin1 = ePin1;
    encoderPin2 = ePin2;

}

void Motor::begin() {
    ledcAttach(forwardPin, robotConfig::DRIVING_FREQUENCY,
               robotConfig::PWM_RESOLUTION);
    ledcAttach(reversePin, robotConfig::DRIVING_FREQUENCY,
               robotConfig::PWM_RESOLUTION);
    ledcWrite(forwardPin, 0);
    ledcWrite(reversePin, 0);

    enableQuadratureEncoder();

    lastSpeedTime = millis();
    lastEncoderCount = encoder.getCount();
}

void Motor::enableQuadratureEncoder() {
    encoder.attachFullQuad(encoderPin1, encoderPin2);
    encoder.setFilter(100);
    encoder.clearCount();
}


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

double Motor::speed() {
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - lastSpeedTime;

    // Only recalculate if at least 20ms have passed.
    // If you check too fast (e.g., 1ms), the deltaCount will be 0 and speed will stutter.
    if (timeElapsed >= robotConfig::MAX_SPEED_SAMPLE_COOLDOWN) {
        
        // 1. Get the current position
        long currentCount = encoder.getCount();
        long deltaCount = currentCount - lastEncoderCount;

        // 2. Convert raw pulses to physical distance (e.g., meters or mm)
        double revolutions = static_cast<double>(deltaCount) / robotConfig::PULSES_REV;
        double distance = revolutions * circumference;  // distance in mm
        
        // 3. Calculate velocity: v = d / t
        double timeSeconds = timeElapsed / 1000.0;
        currentSpeed = distance / timeSeconds;

        // 4. Save current values for the next cycle
        lastEncoderCount = currentCount;
        lastSpeedTime = currentTime;
    }

    // Returns the calculated speed (or the cached speed if < 20ms elapsed)
    return currentSpeed;
}

// test method
void Motor::oneTurn(void) {
    int count = encoder.getCount();
    drive(500, robotConfig::FORWARD);
    while (encoder.getCount()  - count < robotConfig::PULSES_REV * 2 / 3) {
        delay(1);
    }
    drive(400, robotConfig::FORWARD);
    while (encoder.getCount() - count < robotConfig::PULSES_REV * 9 / 10) {
        delay(1);
    }
    drive(350, robotConfig::FORWARD);
    while (encoder.getCount() - count < robotConfig::PULSES_REV ) {
        delay(1);
    }
    
    drive(0, robotConfig::STOPPED);
}
