#pragma once
#include <Arduino.h>

// Placeholder IMU / tilt-sensor interface.
//
// The only capability the mission FSM needs right now is a debounced answer to
// "am I on the ramp (tilted) or on flat ground?". The ramp counter says WHEN to
// expect the ramp; this sensor says WHETHER we are actually on it.
class TiltSensor {
public:
    TiltSensor(int pin);

    void begin();

    // true  = tilted, i.e. on the ramp
    // false = flat ground
    bool isOnRamp();

private:
    int _pin;
};
