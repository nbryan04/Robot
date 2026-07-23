#pragma once
#include <Arduino.h>

class LineFollower {
private:
    int lPin, mPin, rPin;
    int lastError = 0;
    int recentError = 0;
    int error = 0;
    
    // --- Non-blocking timing variables ---
    unsigned long lastUpdateTime = 0;
    const unsigned long UPDATE_INTERVAL = 10; // Read sensors every 10ms (100 Hz)
    
    double currentCorrection = 0.0;
    bool active = false;

public:
    LineFollower(int lSensorPin, int mSensorPin, int rSensorPin);
    
    void begin();
    void start();
    void stop();
    void update();
    
    double getCorrection();
    bool isFollowing();
};