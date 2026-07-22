#pragma once
#include <Arduino.h>

// MPU-6050 based ramp / tilt detector.
//
// Uses the accelerometer to measure the robot's inclination from horizontal.
// The angle is computed as the tilt of the gravity vector away from the sensor's
// Z axis, so it is independent of which way the board is mounted in yaw and works
// whether Z points up or down when the robot is flat.
//
// isOnRamp() latches with hysteresis: it becomes true the first time the tilt
// crosses rampOnAngle (the base of the ramp) and stays true until the tilt drops
// back below rampOffAngle (off the ramp / crest reached).
//
// The MPU-6050 talks I2C (default address 0x68), so it shares the bus with any
// other I2C device (e.g. the OLED) on the same SDA/SCL pins.
class TiltSensor {
public:
    TiltSensor(int sdaPin, int sclPin, uint8_t address = 0x68);

    void begin();
    void update();          // read + smooth + latch; call every loop()

    bool isOnRamp();        // latched ramp state (true = on the incline)
    float getTiltAngle();   // current smoothed tilt from horizontal (deg)

    // ---- Tuning (adjustable) ----
    float rampOnAngle  = 7.125f;  // deg: detect the base of the ramp
    float rampOffAngle = 4.0f;    // deg: detect coming off the ramp / crest

private:
    int _sda;
    int _scl;
    uint8_t _addr;

    float _tiltAngle = 0.0f;   // smoothed inclination from horizontal (deg)
    bool  _onRamp = false;     // latched state
    bool  _seeded = false;     // has the smoother been initialised yet
    unsigned long _lastRead = 0;

    // ---- Fixed constants ----
    static constexpr float SMOOTHING = 0.2f;            // EMA factor on the angle
    static constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +/-2g full-scale
    static constexpr unsigned long READ_INTERVAL_MS = 10;
};
