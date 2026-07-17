#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <ESP32Encoder.h>

#include "config.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "motor.h"

Motor::Motor(int p1, int p2, int ePin1, int ePin2, float diameter, int polarity,
             std::function<int(float)> speedToDutyCallback) {
    
    mapSpeedToDutyCycle = speedToDutyCallback;
    
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
            if (motorState == Reverse || motorState == Stopped) {
                ledcWrite(reversePin, 0);
                delay(robotConfig::PWM_DELAY);
            }
            ledcWrite(forwardPin, dutyCycle);
            motorState = Forward;
            break;

        case (robotConfig::REVERSE):
            if (motorState == Forward || motorState == Stopped) {
                ledcWrite(forwardPin, 0);
                delay(robotConfig::PWM_DELAY);
            }
            ledcWrite(reversePin, dutyCycle);
            motorState = Reverse;
            break;

        case (robotConfig::STOPPED):
            ledcWrite(forwardPin, 0);
            ledcWrite(reversePin, 0);
            delay(robotConfig::PWM_DELAY);
            motorState = Stopped;
            break;

        default:
            ledcWrite(forwardPin, 0);
            ledcWrite(reversePin, 0);
            motorState = Stopped;
            break;
    }
}

void Motor::driveDistance(float distance, float speed) {
    // 1. Calculate ticks using physical constants from robotConfig
    float revolutions = distance / robotConfig::WHEEL_1_CIRCUMFERENCE;
    long ticksToMove = revolutions * robotConfig::PULSES_REV;
    
    // 2. Set absolute target
    targetEncoderCount = encoder.getCount() + ticksToMove;
    
    // 3. Determine direction
    currentDriveDirection = (distance >= 0) ? robotConfig::FORWARD : robotConfig::REVERSE;
    
    // 4. Map physical speed to duty cycle using your new function
    int dutyCycle = mapSpeedToDutyCycle(abs(speed)); 
    
    // 5. Start motor (this safely updates motorState internally)
    drive(dutyCycle, currentDriveDirection);
    
    // 6. Set our new high-level task state
    movementState = Driving; 
}

void Motor::update() {
    // If we aren't executing a distance command, do nothing
    if (movementState == Idle) return;

    long currentCount = encoder.getCount();

    // STATE: Actively driving towards the target distance
    if (movementState == Driving) {
        bool targetReached = false;
        
        if (currentDriveDirection == robotConfig::FORWARD && currentCount >= targetEncoderCount) {
            targetReached = true;
        } else if (currentDriveDirection == robotConfig::REVERSE && currentCount <= targetEncoderCount) {
            targetReached = true;
        }

        if (targetReached) {
            // Initiate Active Braking
            int brakeDirection = (currentDriveDirection == robotConfig::FORWARD) ? robotConfig::REVERSE : robotConfig::FORWARD;
            
            // Hard brake at 50% max duty cycle from config. 
            drive(robotConfig::MAX_DUTY / 2, brakeDirection); 
            
            if (currentDriveDirection == robotConfig::FORWARD) {
                brakeTargetCount = currentCount - 20; 
            } else {
                brakeTargetCount = currentCount + 20; 
            }
            
            movementState = Braking;
        }
    } 
    // STATE: Applying reverse power to brake
    else if (movementState == Braking) {
        bool brakeFinished = false;

        if (currentDriveDirection == robotConfig::FORWARD && currentCount <= brakeTargetCount) {
            brakeFinished = true;
        } else if (currentDriveDirection == robotConfig::REVERSE && currentCount >= brakeTargetCount) {
            brakeFinished = true;
        }

        if (brakeFinished) {
            // Shut motors down
            drive(0, robotConfig::STOPPED); 
            
            // Mark the task as finished
            movementState = Idle;
        }
    }
}


// returns motor speed in m/s
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
        double timeSeconds = timeElapsed;
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
