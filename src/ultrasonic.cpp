#include "ultrasonic.h"

Ultrasonic::Ultrasonic(int trig, int echo) {
    this->trigPin = trig;
    this->echoPin = echo;
}

void Ultrasonic::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    // attachInterruptArg is an ESP32 feature that lets us pass 'this' (the current struct instance) 
    // into the static interrupt handler so it knows which sensor triggered it.
    attachInterruptArg(digitalPinToInterrupt(echoPin), isrHandler, this, CHANGE);
}

// The ESP32 jumps here the microsecond the ECHO pin changes state.
// We keep it ultra-short and fast.
void IRAM_ATTR Ultrasonic::isrHandler(void* arg) {
    // Cast the argument back into an Ultrasonic object and call its specific handler
    Ultrasonic* instance = static_cast<Ultrasonic*>(arg);
    instance->handleInterrupt();
}

void IRAM_ATTR Ultrasonic::handleInterrupt() {
    if (digitalRead(echoPin) == HIGH) {
        // The pulse just started
        echoStart = micros();
    } else {
        // The pulse just ended
        echoEnd = micros();
        newReading = true;
    }
}

void Ultrasonic::update() {
    unsigned long currentTime = millis();
    
    // 1. Fire the trigger non-blockingly (once every PING_INTERVAL)
    if (currentTime - lastPingTime >= PING_INTERVAL) {
        lastPingTime = currentTime;
        
        // The 10 microsecond delay is so infinitesimally small (0.01 ms) 
        // that it doesn't violate our non-blocking rule for the main loop.
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
    }
    
    // 2. If the interrupt caught an echo, calculate the distance
    if (newReading) {
        // Briefly pause interrupts to safely copy the data without it changing mid-copy
        noInterrupts();
        unsigned long duration = echoEnd - echoStart;
        newReading = false;
        interrupts();
        
        // Speed of sound is ~0.0343 cm/microsecond. Divide by 2 because the sound goes out and back.
        float calculatedDistance = (duration * 0.0343) / 2.0;
        
        // Basic sanity check to filter out glitchy 0 or extreme readings
        if (calculatedDistance > 0 && calculatedDistance < 400) { 
            currentDistanceCm = calculatedDistance;
        }
    }
}