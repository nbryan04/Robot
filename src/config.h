#pragma once
#include <Arduino.h>
namespace robotConfig {
// Software Variables
inline constexpr int DRIVING_FREQUENCY = 2000;
inline constexpr int MAX_DUTY = 1023;
inline constexpr int PWM_RESOLUTION = 10;
inline constexpr int MOTOR1_POLARITY = 1;
inline constexpr int MOTOR2_POLARITY = 1;
inline constexpr int DOWNSAMPLING_FACTOR = 10;
inline constexpr int FORWARD = 1;
inline constexpr int REVERSE = -1;
inline constexpr int STOPPED = 0;
inline constexpr int PWM_DELAY = 10;  // delay between switching pins

// Physical Constants
inline constexpr float WHEEL_1_DIAMETER = 79;  //(mm)
inline constexpr float WHEEL_1_CIRCUMFERENCE = PI * WHEEL_1_DIAMETER;
inline constexpr float WHEEL_1_SINGLE_PULSE =
    WHEEL_1_CIRCUMFERENCE / 1441;  //(mm)
inline constexpr int PULSES_REV = 1441;

// Line Following Constants
inline constexpr float LF_KD = 0.95;       // derivative coefficient
inline constexpr float LF_KP = 0.8;        // proportional coefficient
inline constexpr int LF_THRESHOLD = 1500;  // tape sensing

}
