#include "motor.h"
#include "config.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>

//temporary function 
int num;
void count() {
    num += 1;
}


Motor::Motor(int p1, int p2, int ePin1, int ePin2) {
    PWM_pin1 = p1;
    PWM_pin2 = p2;
    encoderPin1 = ePin1;
    encoderPin2 = ePin2;
   };

void Motor::begin() {
    ledcAttach(PWM_pin1, robotConfig::DRIVING_FREQUENCY, robotConfig::PWM_RESOLUTION);
    ledcAttach(PWM_pin2,robotConfig::DRIVING_FREQUENCY, robotConfig::PWM_RESOLUTION);
    attachInterrupt(digitalPinToInterrupt(encoderPin1),count, RISING);
    //attachInterrupt(encoderPin2);
}

void Motor::drive(float speed) {
    
      }

void Motor::drive_distance(float distance, float speed) {
}

//temporary method
void Motor::one_turn(void) {
    num = 0;
    while(num <= 1441) {
	ledcWrite(PWM_pin1, 400);
    }
	ledcWrite(PWM_pin1, 0);
    }


