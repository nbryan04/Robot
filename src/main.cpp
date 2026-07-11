#include "config.h"
#include "motor.h"
#include "claw.h"
#include <Arduino.h>

// Define the ESP32 pins connected to your servos
const int HAND_SERVO_PIN = 47; 
const int ARM_SERVO_PIN = 48;  

Claw myClaw(HAND_SERVO_PIN, ARM_SERVO_PIN);

// Timer variables to pause between grabs
unsigned long lastActionTime = 0;
const unsigned long PAUSE_BETWEEN_GRABS = 2000; // 2 seconds

void setup() {
    Serial.begin(115200);
    myClaw.begin();
    
    Serial.println("Claw initialized. Starting continuous grab loop...");
}

void loop() {
    // 1. You MUST call this every loop iteration to advance the state machine
    myClaw.update();

    // 2. Repeatedly trigger the grab sequence
    if (myClaw.currentState == IDLE) {
        
        // Wait 2 seconds after finishing the last grab before starting the next
        if (millis() - lastActionTime >= PAUSE_BETWEEN_GRABS) {
            Serial.println("Grabbing!");
            myClaw.startGrabSequence();
        }
        
    } else {
        // Keep resetting the timer as long as the claw is moving
        lastActionTime = millis();
    }
}


