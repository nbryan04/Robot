#include "config.h"
#include "motor.h"
#include <Arduino.h>

const int PWM_PIN1 = 3;
const int PWM_PIN2 = 4;
const int ENCODER_PIN1 = 5;
const int ENCODER_PIN2 = 6;
int PWM_FREQ = 2000; // Hz
int PWM_RESOLUTION = 8;

Motor motor1(PWM_PIN1, PWM_PIN2,ENCODER_PIN1, ENCODER_PIN2, robotConfig::MOTOR1_POLARITY);
void setup() {
    motor1.begin();
}

void loop() {
    motor1.one_turn();
    delay(2000);

}


