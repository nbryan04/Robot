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
inline constexpr int MAX_SPEED_SAMPLE_COOLDOWN = 20; // ms
// Physical Constants
inline constexpr float WHEEL_1_DIAMETER = 79;  //(mm)
inline constexpr float WHEEL_1_CIRCUMFERENCE = PI * WHEEL_1_DIAMETER;
inline constexpr float WHEEL_1_SINGLE_PULSE =
    WHEEL_1_CIRCUMFERENCE / 1441;  //(mm)
inline constexpr int PULSES_REV = 5764;

// Line Following Constants
inline constexpr float LF_KD = 0.1;       // derivative coefficient
inline constexpr float LF_KP = 0.06;        // proportional coefficient
inline constexpr int LF_THRESHOLD = 1500;  // tape sensing

//Claw constants
inline constexpr int ARM_UP_ANGLE = 150;
inline constexpr int ARM_DOWN_ANGLE = 10;
inline constexpr int HAND_CLOSE_ANGLE = 3;
inline constexpr int HAND_OPEN_ANGLE = 90;

// IR Sensor Pins (TBD - fill in once wired)
inline constexpr int IR_ADC_PIN    = 5;  // analog input; must be an ADC1 pin (GPIO1-10 on the S3)
inline constexpr int IR_SELECT_PIN = 38;  // HIGH = search 1kHz, LOW = search 10kHz

// Define mapping for the LEFT motor
// Define mapping for the LEFT motor
inline int leftSpeedToDuty(float speed) {
    // Safety check to allow the motor to fully stop
    if (speed <= 0.01f) return 0; 
    
    // Derived from your linear region (PWM 400 to 600)
    int calculatedPWM = (1311.5 * speed) + 342;
    
    // Cap it at max duty just to be safe
    return constrain(calculatedPWM, 0, robotConfig::MAX_DUTY);
}

// Define mapping for the RIGHT motor
inline int rightSpeedToDuty(float speed) {
    if (speed <= 0.01f) return 0;
    
    // Applying the 1.07x multiplier for the right side
    // Slope: 1311.5 * 1.07 = 1403.3
    // Intercept: 342 * 1.07 = ~366
    // int calculatedPWM = (1403.3 * speed) + 366;
    int calculatedPWM = round(leftSpeedToDuty(speed) * 1.07f); // Round to nearest integer
    return constrain(calculatedPWM, 0, robotConfig::MAX_DUTY);

}
}
