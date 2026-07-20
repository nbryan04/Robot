#pragma once
#include <Arduino.h>
#include "esp_adc/adc_continuous.h"

// Detects a sustained 1kHz or 10kHz tone on a filtered analog IR input using the
// Goertzel algorithm. Which frequency it hunts is chosen by a digital select
// pin (HIGH = 1kHz, LOW = 10kHz), latched at the moment startSearch() is called.
//
// Fully non-blocking: hardware (DMA) samples the ADC in the background, and
// update() drains whatever is ready and streams it through Goertzel one sample
// at a time. Call update() every loop().
class IR_Sensor {
public:
    IR_Sensor(int adcPin, int selectPin);

    void begin();        // configure select pin + ADC continuous driver (does not sample yet)
    void startSearch();  // latch target from select pin, start sampling
    void stop();         // stop sampling, return to idle

    void update();       // non-blocking: drain samples + run Goertzel. Call every loop.

    bool  detected() const  { return _state == State::DETECTED; }
    bool  searching() const { return _state == State::SEARCHING; }
    float magnitude() const { return _lastMag; }     // last window's AC amplitude estimate, ~[0,1]
    float threshold() const { return _threshold; }   // amplitude the current target must clear
    float targetFreq() const { return _targetFreq; } // frequency currently being hunted

private:
    enum class State { IDLE, SEARCHING, DETECTED };

    void processSample(float x);
    void evaluateWindow();
    void resetWindow() { _s1 = 0.0f; _s2 = 0.0f; _n = 0; }

    // Detection parameters (kept out of config.h on purpose - only pins live there).
    static constexpr int   SAMPLE_RATE   = 40000;    // Hz; 4 samples/cycle at 10kHz
    static constexpr int   WINDOW_N      = 200;      // samples/window -> 5ms, integer k for both tones
    static constexpr float FREQ_LOW      = 1000.0f;  // select HIGH
    static constexpr float FREQ_HIGH     = 10000.0f; // select LOW
    static constexpr float THRESHOLD_1K  = 0.10f;    // TBD: calibrate via magnitude()
    static constexpr float THRESHOLD_10K = 0.10f;    // TBD: calibrate via magnitude()
    static constexpr int   CONFIRM_COUNT = 4;        // windows in a row above threshold = detected

    int _adcPin;
    int _selectPin;

    adc_continuous_handle_t _handle = nullptr;
    adc_channel_t _channel;

    State _state = State::IDLE;

    // Precomputed Goertzel coefficients for each target.
    float _coeffLow = 0.0f;
    float _coeffHigh = 0.0f;

    // Active window state (chosen at startSearch()).
    float _coeff = 0.0f;
    float _threshold = 0.0f;
    float _targetFreq = 0.0f;

    // Streaming Goertzel accumulators.
    float _s1 = 0.0f;
    float _s2 = 0.0f;
    int   _n = 0;
    int   _hits = 0;
    float _lastMag = 0.0f;
};
