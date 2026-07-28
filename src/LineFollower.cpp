#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include "LineFollower.h"
#include "config.h"

LineFollower::LineFollower() {}

void LineFollower::begin() {
    // No-op: the ADC channels are configured and sampled by IR_Sensor's
    // continuous scan; this class only consumes the raw values via update().
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

void LineFollower::update(int rawLeft, int rawMid, int rawRight) {
    if (!active) return;

    unsigned long currentMillis = millis();

    // Only recompute the error/correction every 10ms.
    if (currentMillis - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = currentMillis;

        bool leftVal = rawLeft > robotConfig::LF_THRESHOLD;
        bool midVal = rawMid > robotConfig::LF_THRESHOLD;
        bool rightVal = rawRight > robotConfig::LF_THRESHOLD;

        // Any sensor over the tape means we've found / are on the line.
        onLine = (leftVal || midVal || rightVal);

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

        // Scale the PD output by 1024 (full PWM duty range) so the fractional
        // LF_KP/LF_KD gains land in motor-PWM units for direct driving.
        currentCorrection = 1024.0 * (robotConfig::LF_KP * error + robotConfig::LF_KD * (error - recentError));
    }
}

double LineFollower::getCorrection() {
    return currentCorrection;
}

bool LineFollower::seesLine() {
    return onLine;
}