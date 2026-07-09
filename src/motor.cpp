#include "config.h"
#include <CircularBuffer.h>
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "motor.h"
#include <Arduino.h>
// TODO:
// PWM control
// Proper implementation of counter (interrupt)
//
Motor::Motor(int p1, int p2, int ePin1, int ePin2, float diameter,
             int polarity) {
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
  encoderCount = 0;
  wheelDiameter = diameter;
}

void Motor::begin() {
  ledcAttach(forwardPin, robotConfig::DRIVING_FREQUENCY,
             robotConfig::PWM_RESOLUTION);
  ledcAttach(backwardsPin, robotConfig::DRIVING_FREQUENCY,
             robotConfig::PWM_RESOLUTION);
  // attachInterrupt(digitalPinToInterrupt(encoderPin1), count, RISING);
  // attachInterrupt(digitalPinToInterrupt(encoderPin2), count, RISING);
  ledcWrite(forwardPin, 0);
  ledcWrite(backwardsPin, 0);
}
void Motor::enableEncoder() {
  encoderEnabled = true;
  attachInterruptArg(
      digitalPinToInterrupt(encoderPin1),
      [](void *arg) IRAM_ATTR {
        Motor *motor = static_cast<Motor *>(arg);
        motor->increaseCount();
      },
      this, RISING);
  attachInterruptArg(
      digitalPinToInterrupt(encoderPin2),
      [](void *arg) IRAM_ATTR {
        Motor *motor = static_cast<Motor *>(arg);
        motor->increaseCount();
      },
      this, RISING);
}

void Motor::disableEncoder() {
  encoderEnabled = false;
  detachInterrupt(digitalPinToInterrupt(encoderPin1));
  detachInterrupt(digitalPinToInterrupt(encoderPin2));
}
void Motor::drive(float speed, int direction) {}

void Motor::drive_distance(float distance, float speed) {}

float Motor::speed() {
  bool wasEnabled = false;
  if (!encoderEnabled) {
    enableEncoder();
    wasEnabled = true;
  }
  int initialTime = micros();
  int initialCount = encoderCount;
  int currentCount = 0;
  int currentTime = 0;
  while (micros() - initialTime < 50) {
    currentCount = encoderCount;
    currentTime = micros();
  }

  if (wasEnabled) {
    disableEncoder();
  }

  return ((currentCount - initialCount) / (currentTime - initialTime)) *
         wheelDiameter / robotConfig::PULSES_REV;
}
void Motor::increaseCount(int num) { encoderCount += 1; }
void Motor::resetCount() { encoderCount = 0; }
