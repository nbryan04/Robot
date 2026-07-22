#include "tilt_sensor.h"
#include <Wire.h>
#include <math.h>

// MPU-6050 registers
static const uint8_t MPU_PWR_MGMT_1  = 0x6B;
static const uint8_t MPU_ACCEL_XOUT_H = 0x3B;

TiltSensor::TiltSensor(int sdaPin, int sclPin, uint8_t address)
    : _sda(sdaPin), _scl(sclPin), _addr(address) {}

void TiltSensor::begin() {
    Wire.begin(_sda, _scl);
    Wire.setClock(400000);

    // Wake the MPU-6050: clear the sleep bit in PWR_MGMT_1.
    Wire.beginTransmission(_addr);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);   // let the sensor settle before the first reading
    update();     // seed the smoothed angle so isOnRamp() is valid immediately
}

void TiltSensor::update() {
    unsigned long now = millis();
    if (now - _lastRead < READ_INTERVAL_MS) return;
    _lastRead = now;

    // Point at the accelerometer registers, then burst-read all six bytes.
    Wire.beginTransmission(_addr);
    Wire.write(MPU_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return;   // repeated start; bail on error
    if (Wire.requestFrom(_addr, (uint8_t)6) != 6) return;

    // Read bytes into named variables: the order of Wire.read() calls inside one
    // expression is unspecified, so we must sequence them explicitly.
    uint8_t xh = Wire.read();
    uint8_t xl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t yl = Wire.read();
    uint8_t zh = Wire.read();
    uint8_t zl = Wire.read();

    int16_t axRaw = (int16_t)((xh << 8) | xl);
    int16_t ayRaw = (int16_t)((yh << 8) | yl);
    int16_t azRaw = (int16_t)((zh << 8) | zl);

    float ax = axRaw / ACCEL_LSB_PER_G;
    float ay = ayRaw / ACCEL_LSB_PER_G;
    float az = azRaw / ACCEL_LSB_PER_G;

    // Inclination from horizontal: angle between gravity and the Z axis.
    // 0 deg when flat, growing as the robot tilts in any direction.
    float horiz = sqrtf(ax * ax + ay * ay);
    float angle = atan2f(horiz, fabsf(az)) * 180.0f / (float)PI;

    // Smooth to reject driving vibration / transient linear acceleration.
    if (!_seeded) {
        _tiltAngle = angle;
        _seeded = true;
    } else {
        _tiltAngle = SMOOTHING * angle + (1.0f - SMOOTHING) * _tiltAngle;
    }

    // Hysteresis latch: trip on the way up at rampOnAngle, release once we drop
    // back below rampOffAngle.
    if (!_onRamp && _tiltAngle >= rampOnAngle) {
        _onRamp = true;
    } else if (_onRamp && _tiltAngle <= rampOffAngle) {
        _onRamp = false;
    }
}

bool TiltSensor::isOnRamp() {
    return _onRamp;
}

float TiltSensor::getTiltAngle() {
    return _tiltAngle;
}
