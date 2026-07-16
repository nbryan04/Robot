#include "config.h"
#include "motor.h"
#include "claw.h"
#include "ultrasonic.h"
#include <Arduino.h>

Ultrasonic mySensor(8, 7); 

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500; 

void setup() {
    Serial.begin(115200);
    mySensor.begin();
    Serial.println("Starting filtered ultrasonic test with edge detection...");
}

void loop() {
    // 1. Keep the background buffer running (Non-blocking)
    mySensor.update();

    // 2. Ask the sensor if an edge was triggered this exact loop
    EdgeEvent currentEvent = mySensor.checkEdgeEvents();

    // 3. React to the edge events INSTANTLY
    if (currentEvent == START_EDGE) {
        Serial.print("\n>>> START EDGE FOUND! Triggered at: ");
        Serial.print(mySensor.filteredDistanceCm);
        Serial.println(" cm <<<");
    } 
    else if (currentEvent == END_EDGE) {
        Serial.println(">>> END EDGE FOUND! Object cleared. <<<\n");
    }

    // 4. Print the live distance at your 500ms interval
    if (millis() - lastPrintTime >= PRINT_INTERVAL) {
        lastPrintTime = millis();
        
        if (mySensor.filteredDistanceCm > 0) {
            Serial.print("Raw: ");
            Serial.print(mySensor.currentDistanceCm);
            Serial.print(" cm  |  Filtered: ");
            Serial.print(mySensor.filteredDistanceCm);
            Serial.println(" cm");
        }
    }
}