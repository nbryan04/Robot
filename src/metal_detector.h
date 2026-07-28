#pragma once
#include <Arduino.h>
#include <atomic>
#include "driver/pcnt.h"   // legacy PCNT — the SAME driver ESP32Encoder uses

// Uses the legacy PCNT driver so it shares the hardware cleanly with the
// encoders (which use ESP32Encoder). The encoders take units 0 and 1, so this
// detector defaults to unit 2 to avoid any collision. Do NOT mix this with the
// newer driver/pulse_cnt.h in the same firmware.
class MetalDetector {
private:
    int sensorPin;
    pcnt_unit_t pcntUnit;         // hardware unit (kept off the encoders' 0/1)
    unsigned long updateInterval;
    float threshold;

    // --- Overflow tracking (counter auto-resets at the high limit) ---
    std::atomic<int> overflow_count{0};

    // --- Moving Average Filter ---
    static const int FILTER_SIZE = 3;
    float freqBuffer[FILTER_SIZE];
    int bufferIndex = 0;
    float freqSum = 0.0;

    float baseFrequency = 0.0;
    float currentFrequency = 0.0;
    float frequencyShift = 0.0;
    unsigned long lastUpdateTime = 0;

public:
    // Constructor allows you to tune the interval, sensitivity, and PCNT unit.
    MetalDetector(int pin, unsigned long interval = 100, float thresholdHz = 350,
                  pcnt_unit_t unit = PCNT_UNIT_2);

    void begin();
    void update();

    // Calibration methods
    void tare();
    void recalibrate(unsigned long sampleTimeMs = 1000);

    // Data getters
    float getShift();
    float getBaseFrequency();
    float getCurrentFrequency();
    bool isMetalDetected();
};
