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
    float Kp_sync = .2; 
    float Kv_sync = 7000; 
    int brakeTicks = 20; // Number of ticks to reverse for active braking

    Drivetrain(Motor& left, Motor& right);

    void driveStraight(float distanceMM, float speed);
    void turn(float degrees, float speed);
    void stop();
    void update();

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