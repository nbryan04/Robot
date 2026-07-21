#pragma once
#include <Arduino.h>

// Placeholder metal-detector interface. Real sensing is TBD.
//
// Flow expected by the mission FSM:
//   - getReferenceFrequency() is read ONCE per rock while the claw is still
//     upright (no metal near the coil) to capture a baseline.
//   - getReading() is read with the claw lowered onto the rock; the FSM
//     compares it against the stored reference to decide metal vs. decoy.
class MetalDetector {
public:
    MetalDetector(int pin);

    void begin();

    // Baseline taken with the claw up, before lowering onto the rock.
    float getReferenceFrequency();

    // Live reading taken with the claw lowered next to the rock.
    float getReading();

private:
    int _pin;
};
