#include "tilt_sensor.h"

TiltSensor::TiltSensor(int pin) : _pin(pin) {}

void TiltSensor::begin() {
    // TODO: configure the real IMU / tilt switch once wired.
    // pinMode(_pin, INPUT);
}

bool TiltSensor::isOnRamp() {
    // TODO: return the real (debounced) tilt state.
    // Placeholder always reports "flat"; the ramp counter still drives the
    // approach, and ramp recovery falls through to the crest, so the mission
    // still progresses to the UPPER deck for structural testing.
    return false;
}
