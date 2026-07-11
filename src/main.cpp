#include "config.h"
#include "motor.h"
#include <Arduino.h>

const int PWM_PIN1_M1 = 12;
const int PWM_PIN2_M1 = 13;
const int PWM_PIN1_M2 = 10;
const int PWM_PIN2_M2 = 11;
const int ENCODER_PIN1_M1 = 6;
const int ENCODER_PIN1_M2 = 5;
int PWM_FREQ = 2000; // Hz
int PWM_RESOLUTION = 10;

Motor motor1(PWM_PIN1_M1, PWM_PIN2_M1, ENCODER_PIN1_M1, robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR1_POLARITY);
Motor motor2(PWM_PIN1_M2, PWM_PIN2_M2, ENCODER_PIN1_M2, robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR2_POLARITY);
void setup() {
    motor1.begin();
    motor2.begin();
    Serial.begin(115200);
}


void loop() {
    motor2.drive(0, robotConfig::STOPPED);
    motor1.drive(700, robotConfig::FORWARD);
    delay(100);
    Serial.println(motor1.speed(9));
}


