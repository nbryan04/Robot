#include "ultrasonic.h"

Ultrasonic::Ultrasonic(int trig, int echo) {
    this->trigPin = trig;
    this->echoPin = echo;
    // Zero out the array on startup
    for (int i = 0; i < FILTER_SIZE; i++) {
        readings[i] = 0.0;
    }
}

void Ultrasonic::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    attachInterruptArg(digitalPinToInterrupt(echoPin), isrHandler, this, CHANGE);
}

void IRAM_ATTR Ultrasonic::isrHandler(void* arg) {
    Ultrasonic* instance = static_cast<Ultrasonic*>(arg);
    instance->handleInterrupt();
}

void IRAM_ATTR Ultrasonic::handleInterrupt() {
    if (digitalRead(echoPin) == HIGH) {
        echoStart = micros();
    } else {
        echoEnd = micros();
        newReading = true;
    }
}

// Fast insertion sort to find the median without disturbing the chronological buffer
// Simple average of the circular buffer
float Ultrasonic::calculateMean() {
    float sum = 0.0;
    
    // Add up all the readings in the buffer
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += readings[i];
    }
    
    // Divide by the number of readings to get the average
    return sum / FILTER_SIZE;
}

void Ultrasonic::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastPingTime >= PING_INTERVAL) {
        lastPingTime = currentTime;
        
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
    }
    
    if (newReading) {
        noInterrupts();
        unsigned long duration = echoEnd - echoStart;
        newReading = false;
        interrupts();
        
        float rawDistance = (duration * 0.0343) / 2.0;

        float calculatedDistance = (rawDistance * SCALE_MULTIPLIER) + BASE_OFFSET;

        if (calculatedDistance > 0 && calculatedDistance < 400) { 
            currentDistanceCm = calculatedDistance;
            
            // 1. Push the new reading into the circular buffer
            readings[readIndex] = currentDistanceCm;
            readIndex++;
            
            // 2. If we hit the end of the array, loop back to the start
            if (readIndex >= FILTER_SIZE) {
                readIndex = 0;
                bufferFull = true; 
            }
            
            // 3. Update the filtered output (fallback to raw if buffer is still filling)
            if (bufferFull) {
                filteredDistanceCm = calculateMean(); // <-- Changed this
            } else {
                filteredDistanceCm = currentDistanceCm;
            }
        }
    }
}