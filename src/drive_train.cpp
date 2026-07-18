#include "drive_train.h"

Drivetrain::Drivetrain(Motor& left, Motor& right) 
    : leftMotor(left), rightMotor(right) {}

void Drivetrain::stop() {
    leftMotor.drive(0, robotConfig::STOPPED);
    rightMotor.drive(0, robotConfig::STOPPED);
    state = Idle;
}

void Drivetrain::driveStraight(float distanceMM, float speed) {
    float revolutions = distanceMM / robotConfig::WHEEL_1_CIRCUMFERENCE;
    long ticksToMove = revolutions * robotConfig::PULSES_REV;

    leftStartEncoder = leftMotor.encoder.getCount();
    rightStartEncoder = rightMotor.encoder.getCount();

    leftTargetEncoder = leftStartEncoder + ticksToMove;
    rightTargetEncoder = rightStartEncoder + ticksToMove;

    leftDriveDirection = (distanceMM >= 0) ? robotConfig::FORWARD : robotConfig::REVERSE;
    rightDriveDirection = leftDriveDirection;
    
    targetSpeed = abs(speed);

    leftMotor.drive(leftMotor.mapSpeedToDutyCycle(targetSpeed), leftDriveDirection);
    rightMotor.drive(rightMotor.mapSpeedToDutyCycle(targetSpeed), rightDriveDirection);
    
    state = DrivingStraight;
}

void Drivetrain::turn(float degrees, float speed) {
    float trackWidth = 202.5; // (mm) 
    float turningCircumference = PI * trackWidth;
    
    float distanceMM = turningCircumference * (abs(degrees) / 360.0f);

    float revolutions = distanceMM / robotConfig::WHEEL_1_CIRCUMFERENCE;
    long ticksToMove = revolutions * robotConfig::PULSES_REV;

    leftStartEncoder = leftMotor.encoder.getCount();
    rightStartEncoder = rightMotor.encoder.getCount();

    if (degrees >= 0) { // Clockwise
        leftDriveDirection = robotConfig::FORWARD;
        rightDriveDirection = robotConfig::REVERSE;
        
        leftTargetEncoder = leftStartEncoder + ticksToMove;
        rightTargetEncoder = rightStartEncoder - ticksToMove;
    } else { // Counter-Clockwise
        leftDriveDirection = robotConfig::REVERSE;
        rightDriveDirection = robotConfig::FORWARD;
        
        leftTargetEncoder = leftStartEncoder - ticksToMove;
        rightTargetEncoder = rightStartEncoder + ticksToMove;
    }

    targetSpeed = abs(speed);

    leftMotor.drive(leftMotor.mapSpeedToDutyCycle(targetSpeed), leftDriveDirection);
    rightMotor.drive(rightMotor.mapSpeedToDutyCycle(targetSpeed), rightDriveDirection);
    
    state = Turning;
}

void Drivetrain::update() {
    if (state == Idle) return;

    long leftCurrent = leftMotor.encoder.getCount();
    long rightCurrent = rightMotor.encoder.getCount();

    // ---------------------------------------------------------
    // STATE 1: Driving Straight OR Turning (Unified Dynamic Ramp)
    // ---------------------------------------------------------
    if (state == DrivingStraight || state == Turning) {
        
        long remainingTicks = abs(leftTargetEncoder - leftCurrent);
        float activeTargetSpeed = targetSpeed;
        
        // --- THE FIX ---
        // 1. Calculate the total size of the current move
        long totalMoveTicks = abs(leftTargetEncoder - leftStartEncoder);
        
        // 2. The slowdown zone is half a wheel rotation, OR half the total move, 
        // whichever is smaller. This prevents short turns from braking instantly.
        long maxSlowdown = robotConfig::PULSES_REV / 2;
        long slowdownZone = min(maxSlowdown, (totalMoveTicks / 2)); 

        if (remainingTicks < slowdownZone) {
            // 3. Give turning a higher minimum speed to overcome sideways scrubbing friction
            float minSafeSpeed = (state == Turning) ? 0.12f : 0.08f; 
            
            float progress = (float)remainingTicks / slowdownZone; 
            activeTargetSpeed = minSafeSpeed + ((targetSpeed - minSafeSpeed) * progress);
        }
        // ---------------

        int leftBasePWM = leftMotor.mapSpeedToDutyCycle(activeTargetSpeed);
        int rightBasePWM = rightMotor.mapSpeedToDutyCycle(activeTargetSpeed);

        long leftDistanceMoved = abs(leftCurrent - leftStartEncoder);
        long rightDistanceMoved = abs(rightCurrent - rightStartEncoder);
        long posSyncError = leftDistanceMoved - rightDistanceMoved; 

        double leftCurrentSpeed = abs(leftMotor.speed());
        double rightCurrentSpeed = abs(rightMotor.speed());
        double velSyncError = leftCurrentSpeed - rightCurrentSpeed;

        int correction = (posSyncError * Kp_sync) + (velSyncError * Kv_sync);
        
        int leftFinalPWM = constrain(leftBasePWM - correction, 0, robotConfig::MAX_DUTY);
        int rightFinalPWM = constrain(rightBasePWM + correction, 0, robotConfig::MAX_DUTY);

        leftMotor.drive(leftFinalPWM, leftDriveDirection);
        rightMotor.drive(rightFinalPWM, rightDriveDirection);

        bool leftDone = (leftDriveDirection == robotConfig::FORWARD) ? (leftCurrent >= leftTargetEncoder) : (leftCurrent <= leftTargetEncoder);
        bool rightDone = (rightDriveDirection == robotConfig::FORWARD) ? (rightCurrent >= rightTargetEncoder) : (rightCurrent <= rightTargetEncoder);

        if (leftDone || rightDone) {
            leftMotor.drive(0, robotConfig::STOPPED);
            rightMotor.drive(0, robotConfig::STOPPED);
            state = Braking;
        }
    }
    
    // ---------------------------------------------------------
    // STATE 2: Gentle Overshoot Correction (No PD Sync)
    // ---------------------------------------------------------
    else if (state == Braking) {
        long leftOvershoot = leftCurrent - leftTargetEncoder;
        long rightOvershoot = rightCurrent - rightTargetEncoder;

        int deadband = 10; // Ticks of acceptable error tolerance
        
        bool leftNeedsCorrection = abs(leftOvershoot) > deadband;
        bool rightNeedsCorrection = abs(rightOvershoot) > deadband;

        if (leftNeedsCorrection || rightNeedsCorrection) {
            int nudgePWM = 350; 
            
            if (leftNeedsCorrection) {
                leftDriveDirection = (leftOvershoot > 0) ? robotConfig::REVERSE : robotConfig::FORWARD;
                leftMotor.drive(nudgePWM, leftDriveDirection);
            } else {
                leftMotor.drive(0, robotConfig::STOPPED);
            }

            if (rightNeedsCorrection) {
                rightDriveDirection = (rightOvershoot > 0) ? robotConfig::REVERSE : robotConfig::FORWARD;
                // Applying your 1.07 hardware compensation multiplier directly to the nudge torque
                rightMotor.drive((int)(nudgePWM * 1.07f), rightDriveDirection);
            } else {
                rightMotor.drive(0, robotConfig::STOPPED);
            }
            
        } else {
            leftMotor.drive(0, robotConfig::STOPPED);
            rightMotor.drive(0, robotConfig::STOPPED);
            state = Idle;
        }
    }
}