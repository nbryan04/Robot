#include "ultrasonic.h"

Ultrasonic::Ultrasonic(int trig, int echo) {
    this->trigPin = trig;
    this->echoPin = echo;
    // Zero out both arrays on startup
    for (int i = 0; i < FILTER_SIZE; i++) {
        readings[i] = 0.0;
        filteredHistory[i] = 0.0; 
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

// Wipe the filter + history buffers and re-arm edge detection so a new sweep
// starts clean, with no stale pre-sweep readings able to trigger a false edge.
void Ultrasonic::beginScan() {
    for (int i = 0; i < FILTER_SIZE; i++) {
        readings[i] = 0.0;
        filteredHistory[i] = 0.0;
    }
    readIndex = 0;
    bufferFull = false;
    historyIndex = 0;
    historyFull = false;
    newEdgeDataReady = false;
    debounceCount = 0;
    currentDistanceCm = -1.0;
    filteredDistanceCm = -1.0;
    scanState = WAITING_FOR_OBJECT;
}

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
            
            // 1. Push the new raw reading into the primary circular buffer
            readings[readIndex] = currentDistanceCm;
            readIndex++;
            
            // 2. If we hit the end of the array, loop back to the start
            if (readIndex >= FILTER_SIZE) {
                readIndex = 0;
                bufferFull = true; 
            }
            
            // 3. Update the filtered output and the HISTORY buffer
            if (bufferFull) {
                filteredDistanceCm = calculateMean(); 
                
                // ---> NEW: Push the fresh mean into the history buffer <---
                filteredHistory[historyIndex] = filteredDistanceCm;
                historyIndex = (historyIndex + 1) % FILTER_SIZE;
                
                if (historyIndex == 0) {
                    historyFull = true;
                }
                
                newEdgeDataReady = true; // Signal checkEdgeEvents that a new mean is ready
                
            } else {
                filteredDistanceCm = currentDistanceCm;
            }
        }
    }
}

// Delta-based edge detection using the rolling history window
Ultrasonic::EdgeEvent Ultrasonic::checkEdgeEvents() {
    if (!historyFull || !newEdgeDataReady) {
        return NONE;
    }
        
    newEdgeDataReady = false; 

    float oldDist = filteredHistory[historyIndex];
    int newestIdx = (historyIndex == 0) ? FILTER_SIZE - 1 : historyIndex - 1;
    float currentDist = filteredHistory[newestIdx];
    
    EdgeEvent eventToReturn = NONE;
    
    bool currentIsValid = (currentDist > 0 && currentDist < MAX_VALID_DISTANCE);
    bool oldIsValid = (oldDist > 0 && oldDist < MAX_VALID_DISTANCE);
    
    float delta = 0.0;
    if (currentIsValid && oldIsValid) {
        delta = currentDist - oldDist; // Signed delta
    }

    // State Machine Logic with Debouncing
    switch (scanState) {
        case WAITING_FOR_OBJECT:
            // Condition: Object appeared OR valid background jumped CLOSER
            if (currentIsValid && (!oldIsValid || delta < -EDGE_JUMP_THRESHOLD)) {
                debounceCount++;
                
                if (debounceCount >= DEBOUNCE_THRESHOLD) {
                    scanState = TRACKING_OBJECT;
                    eventToReturn = START_EDGE;
                    debounceCount = 0; // Reset for the next state
                }
            } else {
                // The edge chattered. Reset the counter!
                debounceCount = 0; 
            }
            break;

        case TRACKING_OBJECT:
            // Condition: Object disappeared OR distance jumped FURTHER
            if (!currentIsValid || (oldIsValid && delta > EDGE_JUMP_THRESHOLD)) {
                debounceCount++;
                
                if (debounceCount >= DEBOUNCE_THRESHOLD) {
                    scanState = WAITING_FOR_OBJECT;
                    eventToReturn = END_EDGE;
                    debounceCount = 0; // Reset for the next state
                }
            } else {
                // The edge chattered. Reset the counter!
                debounceCount = 0;
            }
            break;
    }
    
    return eventToReturn;
}