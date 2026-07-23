#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include "LineFollower.h"
#include "config.h"

LineFollower::LineFollower(int lSensorPin, int mSensorPin, int rSensorPin) {
    lPin = lSensorPin;
    mPin = mSensorPin;
    rPin = rSensorPin;
}

void LineFollower::begin() {
    pinMode(lPin, INPUT);
    pinMode(mPin, INPUT);
    pinMode(rPin, INPUT);
}

void LineFollower::start() {
    active = true;
    lastUpdateTime = millis();
}

void LineFollower::stop() {
    active = false;
    currentCorrection = 0.0;
}

bool LineFollower::isFollowing() {
    return active;
}

void LineFollower::update() {
    if (!active) return;

    unsigned long currentMillis = millis();
    
    // Only process the analog reads and math every 10ms
    if (currentMillis - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = currentMillis;

        bool leftVal = analogRead(lPin) > robotConfig::LF_THRESHOLD;
        bool midVal = analogRead(mPin) > robotConfig::LF_THRESHOLD;
        bool rightVal = analogRead(rPin) > robotConfig::LF_THRESHOLD;

        if (!leftVal && midVal && !rightVal) {
            error = 0;
        } else if (leftVal && midVal && !rightVal) {
            error = -1;
        } else if (!leftVal && midVal && rightVal) {
            error = 1;
        } else if (leftVal && !midVal && !rightVal) {
            error = -3;
        } else if (!leftVal && !midVal && rightVal) {
            error = 3;
        } else if (!leftVal && !midVal && !rightVal) {
            if (lastError > 0) error = 5;
            if (lastError <= 0) error = -5;
        }

        if (error != lastError) {
            recentError = lastError;
            lastError = error;
        } 

        currentCorrection = robotConfig::LF_KP * error + robotConfig::LF_KD * (error - recentError);
    }
}

double LineFollower::getCorrection() {
    return currentCorrection;
}