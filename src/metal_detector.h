#pragma once
#include <Arduino.h>
#include <atomic>
#include "driver/pulse_cnt.h" 

class MetalDetector {
private:
    int sensorPin;
    unsigned long updateInterval;
    float threshold; 

    // --- PCNT Hardware Tracking ---
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_channel_handle_t pcnt_chan = NULL;
    std::atomic<int> overflow_count{0};

    // --- Moving Average Filter ---
    static const int FILTER_SIZE = 5; 
    float freqBuffer[FILTER_SIZE];
    int bufferIndex = 0;
    float freqSum = 0.0;

    float baseFrequency = 0.0;
    float currentFrequency = 0.0;
    float frequencyShift = 0.0;
    unsigned long lastUpdateTime = 0;

    // Hardware interrupt callback for when the 16-bit counter maxes out
    static bool IRAM_ATTR pcnt_overflow_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

public:
    // Constructor allows you to tune the interval and sensitivity
    MetalDetector(int pin, unsigned long interval = 500, float thresholdHz = 50.0);

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