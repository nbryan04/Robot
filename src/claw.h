#pragma once


struct Claw {
public:
    // Constructor
    Claw(int hpin, int apin);
    
    // All variables exposed out in the open!
    int hpin;
    int apin;
    enum GrabState {
        IDLE,
        INIT_CLOSE,
        HOVERING,
        OPENING,
        LOWERING,
        GRABBING,
        LIFTING
    };

    GrabState currentState = IDLE;
    unsigned long stateStartTime = 0;
    
    // Core methods
    void setAngle(int pin, int angle);
    void begin();
    
    // Non-blocking sequence methods (replacing the old grabRock)
    void startGrabSequence();
    void update();

    
};
