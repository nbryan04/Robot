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
    Wire.setClock(100000); // Kept at 100kHz for JST connector stability!

    // Probe for the MPU-6050 first. 
    Wire.beginTransmission(_addr);
    _present = (Wire.endTransmission() == 0);
    if (!_present) return;

    // Wake the MPU-6050: clear the sleep bit in PWR_MGMT_1.
    Wire.beginTransmission(_addr);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);   // let the sensor settle before the first reading
    update();     // seed the smoothed angle so isOnRamp() is valid immediately
}

void TiltSensor::update() {
    if (!_present) return;   // no MPU on the bus: stay silent

    unsigned long now = millis();

    // Calculate precise time delta (dt) in seconds for the Gyro integration
    float dt = (now - _lastRead) / 1000.0f;
    
    if (dt < (READ_INTERVAL_MS / 1000.0f)) return;
    _lastRead = now;

    Wire.beginTransmission(_addr);
    Wire.write(MPU_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return;   // repeated start; bail on error
    if (Wire.requestFrom(_addr, (uint8_t)14) != 14) return;

    // 1. Read Accelerometer Data
    uint8_t axh = Wire.read(); uint8_t axl = Wire.read();
    uint8_t ayh = Wire.read(); uint8_t ayl = Wire.read();
    uint8_t azh = Wire.read(); uint8_t azl = Wire.read();

    // 2. Read Temperature Data (Discarded, but must be read to advance the buffer)
    Wire.read(); Wire.read(); 

    // 3. Read Gyroscope Data
    uint8_t gxh = Wire.read(); uint8_t gxl = Wire.read();
    uint8_t gyh = Wire.read(); uint8_t gyl = Wire.read();
    uint8_t gzh = Wire.read(); uint8_t gzl = Wire.read();

    // --- Process Accelerometer ---
    int16_t axRaw = (int16_t)((axh << 8) | axl);
    int16_t ayRaw = (int16_t)((ayh << 8) | ayl);
    int16_t azRaw = (int16_t)((azh << 8) | azl);

    float ax = axRaw / ACCEL_LSB_PER_G;
    float ay = ayRaw / ACCEL_LSB_PER_G;
    float az = azRaw / ACCEL_LSB_PER_G;

    // NEW FORWARD: The Y-axis is now forward.
    // 'ay' replaces 'ax' as the pitch vector, and 'ax' handles the lateral vector.
    float accelPitch = atan2f(-ay, sqrtf(ax * ax + az * az)) * 180.0f / (float)PI;

    // --- Process Gyroscope ---
    // Because forward is Y, pitching the robot up rotates it around the X-axis.
    int16_t gxRaw = (int16_t)((gxh << 8) | gxl);
    
    float gyroRateX = -gxRaw / GYRO_LSB_PER_DEG;

    // --- COMPLEMENTARY FILTER ---
    if (!_seeded) {
        _tiltAngle = accelPitch;
        _seeded = true;
    } else {
        // Fuse the X-axis gyro rate with the Y-axis accelerometer pitch
        _tiltAngle = ALPHA * (_tiltAngle + gyroRateX * dt) + (1.0f - ALPHA) * accelPitch;
    }

    // --- Hysteresis Latch ---
    float absAngle = fabsf(_tiltAngle);
    
    if (!_onRamp && absAngle >= rampOnAngle) {
        _onRamp = true;
    } else if (_onRamp && absAngle <= rampOffAngle) {
        _onRamp = false;
    }
}

bool TiltSensor::isOnRamp() {
    return _onRamp;
}

float TiltSensor::getTiltAngle() {
    return _tiltAngle;
}

bool TiltSensor::isPresent() {
    return _present;
}