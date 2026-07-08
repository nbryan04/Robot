#include "motor.h"
#include "config.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include "LineFollower.h"

LineFollower::LineFollower(int lSensorPin, int mSensorPin, int rSensorPin) {
    lPin = lSensorPin;
    mPin = mSensorPin;
    rPin = rSensorPin;
}

void LineFollower::begin() {}

void LineFollower::update() {
    int leftVal = analogRead(lPin);
    int midVal = analogRead(mPin);
    int rightVal = analogRead(rPin);

    
}