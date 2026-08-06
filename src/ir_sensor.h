#pragma once
#include <Arduino.h>
#include "esp_adc/adc_continuous.h"

// Detects a sustained 1kHz or 10kHz tone on a filtered analog IR input using the
// Goertzel algorithm. Which frequency it hunts is chosen by a digital select
// pin (HIGH = 1kHz, LOW = 10kHz), latched at the moment startSearch() is called.
//
// This one driver also owns the three line-follower analog channels, folded into
// the same continuous (DMA) scan so the IR and the line follower share ADC1 with
// NO one-shot/continuous conflict. The scan pattern samples the IR channel on
// every other slot -> the IR keeps a uniform 40kHz stream (Goertzel unaffected),
// while the LF channels share the rest (~13kHz each, far more than they need).
//
// Fully non-blocking: hardware (DMA) samples the ADC in the background, and
// update() drains whatever is ready -- streaming IR samples through Goertzel and
// latching the latest raw value for each LF channel. Call update() every loop().
class IR_Sensor {
public:
    // adcPin = IR beacon input (ADC1). selectPin = 1kHz/10kHz select (digital).
    // lfLeft/Mid/RightPin = the three line-follower channels (all ADC1).
    IR_Sensor(int adcPin, int selectPin, int lfLeftPin, int lfMidPin, int lfRightPin);

    void begin();        // configure select pin + ADC continuous driver (does not sample yet)
    void startSearch();  // latch target from select pin, start sampling (IR + LF)
    void stop();         // stop sampling, return to idle

    // The two competition surfaces use different 10kHz detection thresholds:
    // surface 1 slightly higher, surface 2 the default. Call before startSearch();
    // if never called it stays on the surface-2 (default) threshold. Only affects
    // the 10kHz tone -- the 1kHz threshold is unchanged.
    void useSurface1Threshold(bool on) { _threshold10k = on ? THRESHOLD_10K_S1 : THRESHOLD_10K; }

    void update();       // non-blocking: drain samples, run Goertzel, latch LF values

    bool  detected() const  { return _state == State::DETECTED; }
    bool  searching() const { return _state == State::SEARCHING; }
    bool  sampling() const  { return _state != State::IDLE; }
    float magnitude() const { return _lastMag; }     // last window's AC amplitude estimate, ~[0,1]
    float threshold() const { return _threshold; }   // amplitude the current target must clear
    float targetFreq() const { return _targetFreq; } // frequency currently being hunted

    // Latest raw ADC value (0..4095) for each line-follower channel, from the scan.
    int lfLeftRaw()  const { return _lfRaw[0]; }
    int lfMidRaw()   const { return _lfRaw[1]; }
    int lfRightRaw() const { return _lfRaw[2]; }

private:
    enum class State { IDLE, SEARCHING, DETECTED };

    void processSample(float x);
    void evaluateWindow();
    void resetWindow() { _s1 = 0.0f; _s2 = 0.0f; _n = 0; }

    // Detection parameters (kept out of config.h on purpose - only pins live there).
    static constexpr int   IR_SAMPLE_RATE = 40000;    // IR effective rate (Goertzel); 4 samples/cycle at 10kHz
    static constexpr int   ADC_TOTAL_RATE = 80000;    // driver total: IR is half the slots -> 40kHz, under the ~83kHz cap
    static constexpr int   PATTERN_LEN    = 6;        // [IR, L, IR, M, IR, R]: IR on every other slot stays uniform
    static constexpr int   WINDOW_N      = 200;       // samples/window -> 5ms, integer k for both tones
    static constexpr float FREQ_LOW      = 1000.0f;   // select HIGH
    static constexpr float FREQ_HIGH     = 10000.0f;  // select LOW
    static constexpr float THRESHOLD_1K  = 0.08f;     // TBD: calibrate via magnitude()
    static constexpr float THRESHOLD_10K = 0.025f;    // 10kHz, surface 2 / default
    static constexpr float THRESHOLD_10K_S1 = 0.025f; // 10kHz, surface 1: slightly higher
    static constexpr int   CONFIRM_COUNT = 4;         // windows in a row above threshold = detected

    int _adcPin;
    int _selectPin;
    int _lfPin[3];

    adc_continuous_handle_t _handle = nullptr;
    adc_channel_t _irChannel;
    adc_channel_t _lfChannel[3];
    int _lfRaw[3] = {0, 0, 0};

    State _state = State::IDLE;

    // Precomputed Goertzel coefficients for each target.
    float _coeffLow = 0.0f;
    float _coeffHigh = 0.0f;

    // Active window state (chosen at startSearch()).
    float _coeff = 0.0f;
    float _threshold = 0.0f;
    float _threshold10k = THRESHOLD_10K;  // active 10kHz threshold (surface-dependent)
    float _targetFreq = 0.0f;

    // Streaming Goertzel accumulators.
    float _s1 = 0.0f;
    float _s2 = 0.0f;
    int   _n = 0;
    int   _hits = 0;
    float _lastMag = 0.0f;
};
