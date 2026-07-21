#include "claw.h"
#include "config.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>

Claw::Claw(int hpin, int apin){
    this->hpin = hpin;
    this ->apin = apin;
}

void Claw::begin() {
    // Standard servo setup: 50Hz frequency, 14-bit resolution (values from 0 to 16383)
    // Note: This uses the newer ESP32 Arduino Core 3.x syntax.
    ledcAttach(hpin, 50, 14);
    ledcAttach(apin, 50, 14);
    
    // Set initial resting states
    setAngle(apin, robotConfig::ARM_UP_ANGLE);
    setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
}

// Helper method to handle the PWM math (add 'void setAngle(int pin, int angle);' to your claw.h)
void Claw::setAngle(int pin, int angle) {
    // Constrain angle to prevent mechanical binding/stripping the gears
    angle = constrain(angle, 0, 180);
    
    // Standard servos expect pulses between ~500 microseconds (0°) and ~2500 microseconds (180°)
    // 50Hz = 20ms period (20,000 microseconds)
    // 14-bit resolution = max value of 16383
    // 500us duty = (500 / 20000) * 16383 ≈ 410
    // 2500us duty = (2500 / 20000) * 16383 ≈ 2048
    int duty = map(angle, 0, 180, 410, 2048);
    
    ledcWrite(pin, duty);
}

void Claw::startGrabSequence() {
    if (currentState == IDLE) {
        currentState = INIT_CLOSE;
        stateStartTime = millis();
        setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
    }
}

void Claw::update() {
    // Advance any granular staged action (independent of the GrabState machine).
    if (actionSeq != ACT_NONE) updateAction();

    if (currentState == IDLE) return;

    unsigned long currentTime = millis();

    switch (currentState) {
        
        case INIT_CLOSE:
            if (currentTime - stateStartTime >= 400) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE + 45);
                currentState = HOVERING;
                stateStartTime = currentTime; 
            }
            break;

        case HOVERING:
            if (currentTime - stateStartTime >= 600) {
                setAngle(hpin, robotConfig::HAND_OPEN_ANGLE);
                currentState = OPENING;
                stateStartTime = currentTime;
            }
            break;

        case OPENING:
            if (currentTime - stateStartTime >= 400) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE);
                currentState = LOWERING;
                stateStartTime = currentTime;
            }
            break;

        case LOWERING:
            if (currentTime - stateStartTime >= 400) {
                setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
                currentState = GRABBING;
                stateStartTime = currentTime;
            }
            break;

        case GRABBING:
            if (currentTime - stateStartTime >= 400) {
                setAngle(apin, robotConfig::ARM_UP_ANGLE);
                currentState = LIFTING;
                stateStartTime = currentTime;
            }
            break;

        case LIFTING:
            if (currentTime - stateStartTime >= 800) {
                currentState = IDLE;
            }
            break;
            
        case IDLE:
            break;
    }
}

// ---- Granular non-blocking actions used by the mission FSM ----------------
// Staged sequences that mimic startGrabSequence()'s stage timings exactly.
// Stage delays (ms), matching the GrabState machine above:
static constexpr unsigned long HOVER_MS = 400;  // INIT_CLOSE: close -> hover
static constexpr unsigned long OPEN_MS  = 600;  // HOVERING:   hover -> OPEN hand
static constexpr unsigned long DOWN_MS  = 400;  // OPENING:    open  -> arm down
static constexpr unsigned long GRAB_MS  = 400;  // LOWERING:   down  -> close/grab
static constexpr unsigned long LIFT_MS  = 800;  // LIFTING:    arm up settle

void Claw::lowerToHover() {
    actionSeq = ACT_LOWER_HOVER;
    actionStep = 0;
    actionStepTime = millis();
}

void Claw::lowerToRock() {
    actionSeq = ACT_LOWER_ROCK;
    actionStep = 0;
    actionStepTime = millis();
}

void Claw::closeHand() {
    actionSeq = ACT_CLOSE;
    actionStep = 0;
    actionStepTime = millis();
}

void Claw::storeToBasket() {
    actionSeq = ACT_STORE;
    actionStep = 0;
    actionStepTime = millis();
}

void Claw::raiseToRest() {
    actionSeq = ACT_RAISE;
    actionStep = 0;
    actionStepTime = millis();
}

bool Claw::actionBusy() {
    return actionSeq != ACT_NONE;
}

void Claw::updateAction() {
    unsigned long now = millis();

    switch (actionSeq) {

        // Lower to the HOVER position. Mirrors INIT_CLOSE -> HOVERING ->
        // OPENING: the hand does NOT open until the arm has reached the hover
        // angle (ARM_DOWN_ANGLE + 45). Stops at hover with the hand open so the
        // mission can baseline the metal detector here, clear of the rock and
        // the metal at the back of the robot.
        case ACT_LOWER_HOVER:
            if (actionStep == 0) {                          // ensure hand closed
                setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
                actionStep = 1;
                actionStepTime = now;
            } else if (actionStep == 1 && now - actionStepTime >= HOVER_MS) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE + 45);  // hover
                actionStep = 2;
                actionStepTime = now;
            } else if (actionStep == 2 && now - actionStepTime >= OPEN_MS) {
                setAngle(hpin, robotConfig::HAND_OPEN_ANGLE);      // open at hover
                actionStep = 3;
                actionStepTime = now;
            } else if (actionStep == 3 && now - actionStepTime >= DOWN_MS) {
                actionSeq = ACT_NONE;                              // at hover, open
            }
            break;

        // Continue from hover down onto the rock (hand stays open).
        case ACT_LOWER_ROCK:
            if (actionStep == 0) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE);       // hover -> down
                actionStep = 1;
                actionStepTime = now;
            } else if (actionStep == 1 && now - actionStepTime >= GRAB_MS) {
                actionSeq = ACT_NONE;                              // at the rock
            }
            break;

        // Grab: close the hand (mirrors LOWERING -> GRABBING).
        case ACT_CLOSE:
            if (actionStep == 0) {
                setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);
                actionStep = 1;
                actionStepTime = now;
            } else if (actionStep == 1 && now - actionStepTime >= GRAB_MS) {
                actionSeq = ACT_NONE;
            }
            break;

        // Store: raise the arm (holding the rock), then open to drop it.
        case ACT_STORE:
            if (actionStep == 0) {
                setAngle(apin, robotConfig::ARM_UP_ANGLE);
                actionStep = 1;
                actionStepTime = now;
            } else if (actionStep == 1 && now - actionStepTime >= LIFT_MS) {
                setAngle(hpin, robotConfig::HAND_OPEN_ANGLE);      // release
                actionStep = 2;
                actionStepTime = now;
            } else if (actionStep == 2 && now - actionStepTime >= GRAB_MS) {
                actionSeq = ACT_NONE;
            }
            break;

        // Decoy (no metal): do NOT close at the bottom or we would pick up the
        // rock. Raise to hover with the hand still OPEN (below the arch, so open
        // is safe and the rock is left behind), close ONLY at hover (clear of the
        // rock), then raise to rest with the hand closed to clear the arch.
        case ACT_RAISE:
            if (actionStep == 0) {
                setAngle(apin, robotConfig::ARM_DOWN_ANGLE + 45);  // up to hover, still open
                actionStep = 1;
                actionStepTime = now;
            } else if (actionStep == 1 && now - actionStepTime >= HOVER_MS) {
                setAngle(hpin, robotConfig::HAND_CLOSE_ANGLE);     // close at hover only
                actionStep = 2;
                actionStepTime = now;
            } else if (actionStep == 2 && now - actionStepTime >= GRAB_MS) {
                setAngle(apin, robotConfig::ARM_UP_ANGLE);         // clear the arch, closed
                actionStep = 3;
                actionStepTime = now;
            } else if (actionStep == 3 && now - actionStepTime >= LIFT_MS) {
                actionSeq = ACT_NONE;
            }
            break;

        case ACT_NONE:
            break;
    }
}
