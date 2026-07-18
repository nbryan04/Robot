#include "claw.h"
#include "config.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>

Claw::Claw(int hpin, int apin){
    this->hpin = hpin;
    this ->apin = apin;
}

void Claw::begin() {
    // Standard servo setup: 50Hz frequency, 14-bit resolution (values from 0 to 16383)
    // Note: This uses the newer ESP32 Arduino Core 3.x syntax.
    ledcAttach(hpin, 50, 14);
    ledcAttach(apin, 50, 14);
    
    // Set initial resting states
    setAngle(apin, robotConfig::ARM_UP_ANGLE);
    setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
}

// Helper method to handle the PWM math (add 'void setAngle(int pin, int angle);' to your claw.h)
void Claw::setAngle(int pin, int angle) {
    // Constrain angle to prevent mechanical binding/stripping the gears
    angle = constrain(angle, 0, 180);
    
    // Standard servos expect pulses between ~500 microseconds (0°) and ~2500 microseconds (180°)
    // 50Hz = 20ms period (20,000 microseconds)
    // 14-bit resolution = max value of 16383
    // 500us duty = (500 / 20000) * 16383 ≈ 410
    // 2500us duty = (2500 / 20000) * 16383 ≈ 2048
    int duty = map(angle, 0, 180, 410, 2048);
    
    ledcWrite(pin, duty);
}

void Claw::startGrabSequence() {
    if (currentState == IDLE) {
        currentState = INIT_CLOSE;
        stateStartTime = millis();
        setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
    }
}

void Claw::update() {
    if (currentState == IDLE) return;

    unsigned long currentTime = millis();

    switch (currentState) {
        
        case INIT_CLOSE:
            if (currentTime - stateStartTime >= 400) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE + 45);
                currentState = HOVERING;
                stateStartTime = currentTime; 
            }
            break;

        case HOVERING:
            if (currentTime - stateStartTime >= 600) {
                setAngle(hpin, robotConfig::HAND_OPEN_ANGLE);
                currentState = OPENING;
                stateStartTime = currentTime;
            }
            break;

        case OPENING:
            if (currentTime - stateStartTime >= 400) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE);
                currentState = LOWERING;
                stateStartTime = currentTime;
            }
            break;

        case LOWERING:
            if (currentTime - stateStartTime >= 400) {
                setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
                currentState = GRABBING;
                stateStartTime = currentTime;
            }
            break;

        case GRABBING:
            if (currentTime - stateStartTime >= 400) {
                setAngle(apin, robotConfig::ARM_UP_ANGLE);
                currentState = LIFTING;
                stateStartTime = currentTime;
            }
            break;

        case LIFTING:
            if (currentTime - stateStartTime >= 800) {
                currentState = IDLE;
            }
            break;
            
        case IDLE:
            break;
    }
}
