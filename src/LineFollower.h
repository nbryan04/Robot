#pragma once
#include <Arduino.h>

// Three-sensor line follower LOGIC. It no longer reads the ADC itself -- the raw
// channel values are sampled by IR_Sensor's continuous DMA scan and passed into
// update(), so the line follower and the IR beacon detector share ADC1 with no
// one-shot/continuous conflict.
class LineFollower {
private:
    int lastError = 0;
    int recentError = 0;
    int error = 0;

    // --- Non-blocking timing variables ---
    unsigned long lastUpdateTime = 0;
    const unsigned long UPDATE_INTERVAL = 10; // recompute error every 10ms (100 Hz)

    double currentCorrection = 0.0;
    bool active = false;
    bool onLine = false;   // did the last sensor read see the tape at all

public:
    LineFollower();

    void begin();   // no-op now (ADC is owned by IR_Sensor's continuous scan)
    void start();
    void stop();
    // Feed the latest raw ADC values (0..4095) for the left/mid/right channels,
    // e.g. from IR_Sensor::lfLeftRaw()/lfMidRaw()/lfRightRaw().
    void update(int rawLeft, int rawMid, int rawRight);

    double getCorrection();
    bool isFollowing();
    bool seesLine();   // true if any sensor is currently over the tape
};
