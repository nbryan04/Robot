#include "metal_detector.h"

MetalDetector::MetalDetector(int pin) : _pin(pin) {}

void MetalDetector::begin() {
    // TODO: configure the real detector once wired.
    // pinMode(_pin, INPUT);
}

float MetalDetector::getReferenceFrequency() {
    // TODO: return the real baseline frequency/reading with the claw upright.
    return 0.0f;
}

float MetalDetector::getReading() {
    // TODO: return the real live reading with the claw lowered onto the rock.
    return 0.0f;
}
