#pragma once
#include <Arduino.h>

struct Ultrasonic {
public:
    Ultrasonic(int trig, int echo);

    // What is the sensor currently doing? (Internal memory)
    enum EdgeState {
        WAITING_FOR_OBJECT,
        TRACKING_OBJECT
    };

    // What just happened this exact millisecond? (External report)
    enum EdgeEvent {
        NONE,
        START_EDGE,
        END_EDGE
    };
    
    static constexpr int FILTER_SIZE = 3; 
    static constexpr float SCALE_MULTIPLIER = 0.97;
    static constexpr float BASE_OFFSET = -0.47; 
    static constexpr float MAX_VALID_DISTANCE = 20.0; 
    static constexpr float EDGE_JUMP_THRESHOLD = 4.0;
    static constexpr int DEBOUNCE_THRESHOLD = 1;

    int trigPin;
    int echoPin;
    
    // Distance variables
    float currentDistanceCm = -1.0;    // The raw, jumpy reading
    float filteredDistanceCm = -1.0;   // The rock-solid median reading
    
    // Circular buffer for the median filter
    float readings[FILTER_SIZE];
    int readIndex = 0;
    bool bufferFull = false;
    
    // Timer variables
    unsigned long lastPingTime = 0;
    const unsigned long PING_INTERVAL = 15; 
    
    // Volatile interrupt variables
    volatile unsigned long echoStart = 0;
    volatile unsigned long echoEnd = 0;
    volatile bool newReading = false;
    
    // Core methods
    EdgeState scanState = WAITING_FOR_OBJECT;
    
    void begin();
    void update();
    void beginScan();   // clear filter + history and re-arm edge detection for a fresh sweep
    float calculateMean();
    
    // The new reporting function
    EdgeEvent checkEdgeEvents(); 
    
    static void IRAM_ATTR isrHandler(void* arg);
    void handleInterrupt();

    float filteredHistory[FILTER_SIZE];
    int historyIndex = 0;
    bool historyFull = false; 
    bool newEdgeDataReady = false;
    int debounceCount = 0;
};
