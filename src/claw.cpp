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
    setAngle(hpin, robotConfig::HAND_OPEN_ANGLE);
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

void Claw::grabRock() {
    // 1. Move arm down
    setAngle(apin, robotConfig::ARM_DOWN_ANGLE);
    delay(800); // Give the heavy MG996R time to physically move
    
    // 2. Close the hand claw
    setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
    delay(400); // Give the smaller MG90S time to grip the rock
    
    // 3. Move the arm back up
    setAngle(apin, robotConfig::ARM_UP_ANGLE);
    delay(800); // Lifting a rock takes effort, let it finish moving
    
    // 4. Reopen the hand
    setAngle(hpin, robotConfig::HAND_OPEN_ANGLE);
    delay(400); // Drop the rock!
}
