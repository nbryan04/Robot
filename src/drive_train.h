#pragma once
#include <Arduino.h>
#include "motor.h"
#include "config.h"

struct Drivetrain {
    Motor& leftMotor;
    Motor& rightMotor;

    enum DriveState { Idle, DrivingStraight, Turning, Braking };
    DriveState state = Idle;

    // Encoder targeting
    long leftTargetEncoder = 0;
    long rightTargetEncoder = 0;
    long leftBrakeTarget = 0;
    long rightBrakeTarget = 0;
    
    // Initial encoder counts when a move starts
    long leftStartEncoder = 0;
    long rightStartEncoder = 0;

    // Independent direction tracking
    int leftDriveDirection = robotConfig::STOPPED;
    int rightDriveDirection = robotConfig::STOPPED;
    int leftBrakeDirection = robotConfig::STOPPED;
    int rightBrakeDirection = robotConfig::STOPPED;
    
    float targetSpeed = 0.0;

    // Tuning constants
    float Kp_sync = 5;
    float Kv_sync = 7000;
    int brakeTicks = 20; // Number of ticks to reverse for active braking

    // Stall recovery: if neither encoder moves for STALL_TIMEOUT_MS during a
    // straight/turn, ramp an extra PWM boost until motion resumes (or the cap),
    // then reset once it frees. Cleared whenever the wheels are turning normally.
    unsigned long STALL_TIMEOUT_MS = 3000;       // no motion this long -> start boosting
    unsigned long STALL_BOOST_INTERVAL_MS = 400; // add another boost step this often while stalled
    int STALL_TICKS = 10;       // encoder change under this (either wheel) = "not moving"
    int STALL_BOOST_STEP = 80;  // extra PWM added per boost step
    int STALL_MAX_BOOST = 400;  // cap on the extra PWM

    // Stall-recovery runtime state.
    long stallRefLeft = 0;
    long stallRefRight = 0;
    unsigned long stallSinceMs = 0;
    unsigned long stallLastBoostMs = 0;
    int stallBoost = 0;

    Drivetrain(Motor& left, Motor& right);

    void driveStraight(float distanceMM, float speed);
    void turn(float degrees, float speed);
    void stop();
    void update();

    // True if the active move is stalled out: stall recovery has kicked in (the
    // PWM boost is engaged) and there has STILL been no encoder motion for at
    // least `ms`. Lets a caller abandon a move that can't complete instead of
    // grinding on it forever.
    bool stalledOut(unsigned long ms) {
        return (state != Idle) && (stallBoost > 0) && (millis() - stallSinceMs >= ms);
    }

    // True if the active move has had no encoder motion for at least `ms` -- a
    // fast contact/stall check that does NOT wait for the stall-recovery boost.
    // Used to drive into an obstacle (e.g. the panel) and detect the touch.
    bool notMovingFor(unsigned long ms) {
        return (state != Idle) && (millis() - stallSinceMs >= ms);
    }

    // Fraction (0..1) of the active driveStraight/turn move completed.
    // Used by the mission FSM to estimate heading during a sweep.
    float moveProgress();

    // Signed distance (mm) the left wheel has travelled since the current move
    // started. Meaningful for straight moves; for a turn it is the pivot arc.
    float lastMoveDistanceMM();

    // Absolute forward (centre-line) odometer in mm, averaged over BOTH wheels.
    // A turn spins the wheels equal-and-opposite, so it cancels in the average
    // and only real forward/back translation accumulates. Snapshot it, drive
    // (including turns), read it again: the difference is net forward distance --
    // exactly what the mission reverses to realign after a sweep/centre.
    float forwardOdometryMM();
};