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
    float trackWidth = 215; // (mm) 
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

float Drivetrain::moveProgress() {
    long total = leftTargetEncoder - leftStartEncoder;
    if (total < 0) total = -total;
    if (total == 0) return 1.0f;

    long done = leftMotor.encoder.getCount() - leftStartEncoder;
    if (done < 0) done = -done;

    float fraction = (float)done / (float)total;
    return (fraction > 1.0f) ? 1.0f : fraction;
}

float Drivetrain::lastMoveDistanceMM() {
    long delta = leftMotor.encoder.getCount() - leftStartEncoder;
    return (float)delta * (robotConfig::WHEEL_1_CIRCUMFERENCE / (float)robotConfig::PULSES_REV);
}

float Drivetrain::forwardOdometryMM() {
    // Average both wheels: on a straight move both counts climb together; on a
    // turn one climbs while the other drops, so the average stays put. What is
    // left is pure forward/back translation of the robot's centre.
    long left  = (long)leftMotor.encoder.getCount();
    long right = (long)rightMotor.encoder.getCount();
    double avgTicks = (left + right) / 2.0;
    return (float)(avgTicks * (robotConfig::WHEEL_1_CIRCUMFERENCE / (double)robotConfig::PULSES_REV));
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
        
        long totalMoveTicks = abs(leftTargetEncoder - leftStartEncoder);
        long maxSlowdown = robotConfig::PULSES_REV / 2;
        long slowdownZone = min(maxSlowdown, (totalMoveTicks / 2)); 

        if (remainingTicks < slowdownZone) {
            float minSafeSpeed = (state == Turning) ? 0.12f : 0.08f; 
            float progress = (float)remainingTicks / slowdownZone; 
            activeTargetSpeed = minSafeSpeed + ((targetSpeed - minSafeSpeed) * progress);
        }

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
            
            targetSpeed = 0.0f; // Flag for settling check
            state = Braking; 
        }
    }
    
    // ---------------------------------------------------------
    // STATE 2: Gentle Overshoot Correction (No PD Sync)
    // ---------------------------------------------------------
    else if (state == Braking) {
        
        // ONE-TIME SETTLING CHECK
        if (targetSpeed == 0.0f) {
            float settleSpeedThreshold = 0.03f;
            if (abs(leftMotor.speed()) > settleSpeedThreshold || abs(rightMotor.speed()) > settleSpeedThreshold) {
                return; 
            }
            targetSpeed = -1.0f; 
        }

        // --- INFER MOVE TYPE (No header changes required) ---
        long expectedLeftDelta = leftTargetEncoder - leftStartEncoder;
        long expectedRightDelta = rightTargetEncoder - rightStartEncoder;
        // If one target went up and the other went down, it was a turn.
        bool wasTurnMove = ((expectedLeftDelta > 0) != (expectedRightDelta > 0));

        // ==========================================
        // CONFIGURABLE BRAKING PARAMETERS
        // ==========================================
        // Straight adjustments
        int straightDeadband = 40;
        int straightNudgePWM = 420;
        float straightRightMultiplier = 1.1f;
        
        // Turn adjustments
        int turnDeadband = 40;
        int turnNudgePWM = 480;
        float turnRightMultiplier = 1.07f;
        // ==========================================

        int activeDeadband = wasTurnMove ? turnDeadband : straightDeadband;
        int activeNudgePWM = wasTurnMove ? turnNudgePWM : straightNudgePWM;
        float activeRightMultiplier = wasTurnMove ? turnRightMultiplier : straightRightMultiplier;

        long leftOvershoot = leftCurrent - leftTargetEncoder;
        long rightOvershoot = rightCurrent - rightTargetEncoder;
        
        bool leftNeedsCorrection = abs(leftOvershoot) > activeDeadband;
        bool rightNeedsCorrection = abs(rightOvershoot) > activeDeadband;

        if (leftNeedsCorrection || rightNeedsCorrection) {
            
            if (leftNeedsCorrection) {
                leftDriveDirection = (leftOvershoot > 0) ? robotConfig::REVERSE : robotConfig::FORWARD;
                leftMotor.drive(activeNudgePWM, leftDriveDirection);
            } else {
                leftMotor.drive(0, robotConfig::STOPPED);
            }

            if (rightNeedsCorrection) {
                rightDriveDirection = (rightOvershoot > 0) ? robotConfig::REVERSE : robotConfig::FORWARD;
                rightMotor.drive((int)(activeNudgePWM * activeRightMultiplier), rightDriveDirection);
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