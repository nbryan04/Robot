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

        // --- Anticipate Overshoot Based on Initial Speed ---
        float expectedOvershootMM = 0.0f;
        
        if (state == DrivingStraight) {
            expectedOvershootMM = (237.5f * targetSpeed);
            if (expectedOvershootMM < 0.0f) expectedOvershootMM = 0.0f; 
            
            float totalMoveMM = (float)totalMoveTicks * (robotConfig::WHEEL_1_CIRCUMFERENCE / (float)robotConfig::PULSES_REV);
            
            float maxAnticipationMM = totalMoveMM - 25.0f;
            if (maxAnticipationMM < 0.0f) maxAnticipationMM = 0.0f; 
            
            float halfMoveMM = totalMoveMM / 2.0f;
            if (maxAnticipationMM > halfMoveMM) {
                maxAnticipationMM = halfMoveMM;
            }

            if (expectedOvershootMM > maxAnticipationMM) {
                expectedOvershootMM = maxAnticipationMM;
            }
        }

        long anticipationTicks = (expectedOvershootMM / robotConfig::WHEEL_1_CIRCUMFERENCE) * robotConfig::PULSES_REV;

        long leftStopThreshold = (leftDriveDirection == robotConfig::FORWARD) ? 
                                 (leftTargetEncoder - anticipationTicks) : 
                                 (leftTargetEncoder + anticipationTicks);
                                 
        long rightStopThreshold = (rightDriveDirection == robotConfig::FORWARD) ? 
                                  (rightTargetEncoder - anticipationTicks) : 
                                  (rightTargetEncoder + anticipationTicks);

        bool leftDone = (leftDriveDirection == robotConfig::FORWARD) ? (leftCurrent >= leftStopThreshold) : (leftCurrent <= leftStopThreshold);
        bool rightDone = (rightDriveDirection == robotConfig::FORWARD) ? (rightCurrent >= rightStopThreshold) : (rightCurrent <= rightStopThreshold);

        if (leftDone || rightDone) {
            leftMotor.drive(0, robotConfig::STOPPED);
            rightMotor.drive(0, robotConfig::STOPPED);
            
            targetSpeed = 0.0f; // Flag for settling check
            state = Braking; 
        }
    }
    
    // ---------------------------------------------------------
    // STATE 2: Gentle Overshoot Correction 
    // ---------------------------------------------------------
    else if (state == Braking) {
        
        // --- INFER MOVE TYPE EARLY ---
        long expectedLeftDelta = leftTargetEncoder - leftStartEncoder;
        long expectedRightDelta = rightTargetEncoder - rightStartEncoder;
        bool wasTurnMove = ((expectedLeftDelta > 0) != (expectedRightDelta > 0));

        // --- PRE-STOPPING COAST PHASE (Active Braking PD) ---
        if (targetSpeed == 0.0f) {
            float settleSpeedThreshold = 0.03f;
            if (abs(leftMotor.speed()) > settleSpeedThreshold || abs(rightMotor.speed()) > settleSpeedThreshold) {
                
                if (!wasTurnMove) {
                    long leftDistanceMoved = abs(leftCurrent - leftStartEncoder);
                    long rightDistanceMoved = abs(rightCurrent - rightStartEncoder);
                    long posSyncError = leftDistanceMoved - rightDistanceMoved; 

                    double leftCurrentSpeed = abs(leftMotor.speed());
                    double rightCurrentSpeed = abs(rightMotor.speed());
                    double velSyncError = leftCurrentSpeed - rightCurrentSpeed;

                    int correction = (posSyncError * Kp_sync) + (velSyncError * Kv_sync);
                    
                    // --- NEW: Initial Guess Baseline ---
                    int initialGuessPWM = 120; // Configurable constant
                    
                    // Left gets a negative base (forces opposite direction)
                    // Right gets a positive base (forces same direction)
                    int leftRawPWM = -initialGuessPWM - correction;
                    int rightRawPWM = initialGuessPWM + correction;

                    // If the math results in a negative PWM, we flip the direction to actively brake
                    int leftCoastDir = leftDriveDirection;
                    if (leftRawPWM < 0) {
                        leftCoastDir = (leftDriveDirection == robotConfig::FORWARD) ? robotConfig::REVERSE : robotConfig::FORWARD;
                    }

                    int rightCoastDir = rightDriveDirection;
                    if (rightRawPWM < 0) {
                        rightCoastDir = (rightDriveDirection == robotConfig::FORWARD) ? robotConfig::REVERSE : robotConfig::FORWARD;
                    }

                    // Apply the absolute power to the assigned direction
                    leftMotor.drive(constrain(abs(leftRawPWM), 0, robotConfig::MAX_DUTY), leftCoastDir);
                    rightMotor.drive(constrain(abs(rightRawPWM), 0, robotConfig::MAX_DUTY), rightCoastDir);
                } 
                else {
                    leftMotor.drive(0, robotConfig::STOPPED);
                    rightMotor.drive(0, robotConfig::STOPPED);
                }

                return; // Wait for speed to drop below threshold
            }
            targetSpeed = -1.0f; 
        }

        // ==========================================
        // CONFIGURABLE BRAKING PARAMETERS
        // ==========================================
        int straightDeadband = 40;
        int straightNudgePWM = 470;
        float straightRightMultiplier = 1.07f;
        
        int turnDeadband = 40;
        int turnNudgePWM = 500;
        float turnRightMultiplier = 1.07f;
        // ==========================================

        int activeDeadband = wasTurnMove ? turnDeadband : straightDeadband;
        int activeNudgePWM = wasTurnMove ? turnNudgePWM : straightNudgePWM;
        float activeRightMultiplier = wasTurnMove ? turnRightMultiplier : straightRightMultiplier;

        long leftOvershoot = leftCurrent - leftTargetEncoder;
        long rightOvershoot = rightCurrent - rightTargetEncoder;
        
        bool leftNeedsCorrection = abs(leftOvershoot) > activeDeadband;
        bool rightNeedsCorrection = abs(rightOvershoot) > activeDeadband;

        // --- ORIGINAL INDEPENDENT NUDGING ---
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