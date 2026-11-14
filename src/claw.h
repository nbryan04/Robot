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

    // ---- Granular non-blocking actions used by the mission FSM ----
    // These are staged sequences that mimic startGrabSequence()'s ordering and
    // timings. Kick one off, then poll actionBusy() until it returns false.
    // Progressed by update(), so update() must be called every loop().
    void lowerToHover();   // close -> hover -> OPEN; stops at hover (hand open)
    void lowerToRock();    // hover -> arm down; stops at the rock (hand open)
    void closeHand();      // grab the rock
    void storeToBasket();  // raise arm, then release into the basket
    void raiseToRest();    // raise arm, then close hand (decoy: back to rest)
    bool actionBusy();     // true while a granular action is still running
    // True once an active raise/store has lifted the arm back off the rock (hover
    // for raiseToRest, top for storeToBasket) -- so the robot can start moving
    // while the claw finishes the rest of the sequence in the background.
    bool armLiftedOffRock();

    // Internal staged-action sequencer (separate from the GrabState machine).
    enum ActionSeq { ACT_NONE, ACT_LOWER_HOVER, ACT_LOWER_ROCK, ACT_CLOSE, ACT_STORE, ACT_RAISE };
    ActionSeq actionSeq = ACT_NONE;
    int actionStep = 0;
    int jitterCount = 0;   // half-cycles done in the store-release jitter
    unsigned long actionStepTime = 0;
    void updateAction();   // called from update() to advance the sequence
};
