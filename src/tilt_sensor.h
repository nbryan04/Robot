#pragma once
#include <Arduino.h>

class TiltSensor {
private:
    int _sda, _scl;
    uint8_t _addr;

    unsigned long _lastRead = 0;
    bool _seeded = false;
    bool _onRamp = false;
    float _tiltAngle = 0.0f; 

    // --- Tuning Parameters ---
    static const unsigned long READ_INTERVAL_MS = 10; 
    
    // The Complementary Filter constant (0.0 to 1.0)
    // 0.98 means: Trust the Gyro 98% (ignores jolts), trust Accel 2% (prevents drift)
    static constexpr float ALPHA = 0.98f; 
    
    static constexpr float ACCEL_LSB_PER_G = 16384.0f; // For +/- 2G range
    static constexpr float GYRO_LSB_PER_DEG = 131.0f;  // For +/- 250 deg/s range

    // Hysteresis thresholds (in degrees)
    float rampOnAngle = 10.0f;
    float rampOffAngle = 5.0f;

public:
    TiltSensor(int sdaPin, int sclPin, uint8_t address = 0x68);
    void begin();
    void update();
    bool isOnRamp();
    float getTiltAngle();
};