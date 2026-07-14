#pragma once
#include <Arduino.h>

 // Use an odd number for a true median

struct Ultrasonic {
public:
    Ultrasonic(int trig, int echo);
    
    int trigPin;
    int echoPin;
    
    // Distance variables
    float currentDistanceCm = -1.0;    // The raw, jumpy reading
    float filteredDistanceCm = -1.0;   // The rock-solid median reading
    
    // Circular buffer for the median filter
    static constexpr int FILTER_SIZE = 5; 
    float readings[FILTER_SIZE];
    int readIndex = 0;
    bool bufferFull = false;
    const float SCALE_MULTIPLIER = 0.97;; // Adjusts the proportional spread
    const float BASE_OFFSET = -0.47;
    
    
    // Timer variables
    unsigned long lastPingTime = 0;
    const unsigned long PING_INTERVAL = 30; 
    
    // Volatile interrupt variables
    volatile unsigned long echoStart = 0;
    volatile unsigned long echoEnd = 0;
    volatile bool newReading = false;
    
    // Core methods
    void begin();
    void update();
    float calculateMean();
    
    // Interrupt handlers
    static void IRAM_ATTR isrHandler(void* arg);
    void handleInterrupt();
};