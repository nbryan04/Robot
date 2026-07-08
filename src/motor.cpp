#include "motor.h"
#include "config.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>

Motor::Motor(int p1, int p2, int ePin1, int ePin2) {
    PWM_pin1 = p1;
    PWM_pin2 = p2;
    encoderPin1 = ePin1;
    encoderPin2 = ePin2;
   };

void Motor::begin() {
    ledcAttach(PWM_pin1, robotConfig::DRIVING_FREQUENCY, robotConfig::PWM_RESOLUTION);
    ledcAttach(PWM_pin2,robotConfig::DRIVING_FREQUENCY, robotConfig::PWM_RESOLUTION);
    attachInterruptArg(digitalPinToInterrupt(encoderPin1), isrWrapper, this, RISING);
    //attachInterrupt(encoderPin2);
}

void IRAM_ATTR Motor::isrWrapper(void* arg) {
    // 1. The hardware hands us a generic 'void*' memory address (arg).
    // 2. We use static_cast to tell the compiler: "Trust me, this is actually a Motor object."
    Motor* instance = static_cast<Motor*>(arg); 
    // 3. Now that it knows it's a Motor, we can increment its specific counter!
    instance->encoderCount += 1; 
}

void Motor::drive(float speed) {
    
      }

void Motor::drive_distance(float distance, float speed) {
}



void Motor::one_turn(void) {
    encoderCount = 0;
    while(encoderCount <= 1441) {
	    ledcWrite(PWM_pin1, 400);
    }
	ledcWrite(PWM_pin1, 0);
}


