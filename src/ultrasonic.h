#pragma once
#include <Arduino.h>

struct Ultrasonic {
public:
    // Constructor
    Ultrasonic(int trig, int echo);
    
    // All variables exposed out in the open!
    int trigPin;
    int echoPin;
    float currentDistanceCm = -1.0; // -1 means no valid reading yet
    
    // Timer variables for the non-blocking trigger
    unsigned long lastPingTime = 0;
    const unsigned long PING_INTERVAL = 50; // Fire a sound pulse every 50ms
    
    // Volatile variables are required because the interrupt changes them in the background
    volatile unsigned long echoStart = 0;
    volatile unsigned long echoEnd = 0;
    volatile bool newReading = false;
    
    // Core methods
    void begin();
    void update();
    
    // Static ISR (Interrupt Service Routine) wrapper required for classes/structs
    static void IRAM_ATTR isrHandler(void* arg);
    
    // The actual interrupt logic for this specific sensor instance
    void handleInterrupt();
};