#include "config.h"
#include "esp32-hal-ledc.h"
#include "motor.h"
#include <Arduino.h>
// TODO:
// PWM control
// Proper implementation of counter (interrupt)
//

Motor::Motor(int p1, int p2, int ePin1, int ePin2, int polarity = 1) {
  if (polarity == -1) { // swap the pins
    forwardPin = p2;
    backwardsPin = p1;
  }

  else {
    forwardPin = p1;
    backwardsPin = p2;
  }
  encoderPin1 = ePin1;
  encoderPin2 = ePin2;
  pulseCount = 0;
  ledcAttach(forwardPin, robotConfig::DRIVING_FREQUENCY,
             robotConfig::PWM_RESOLUTION);
  ledcAttach(backwardsPin, robotConfig::DRIVING_FREQUENCY,
             robotConfig::PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), count, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), count, RISING);
  ledcWrite(forwardPin, 0);
  ledcWrite(backwardsPin, 0);
};

/* void IRAM_ATTR motor1ISR() {
    motor1::increaseCount();
}
void IRAM_ATTR motor2ISR() {} */ //probably a better way to do this?
void Motor::drive(float speed, int direction) {}

void Motor::drive_distance(float distance, float speed) {}

float Motor::speed() {}
void Motor::increaseCount() { pulseCount += 1; }
void Motor::resetCount() { pulseCount = 0; }
