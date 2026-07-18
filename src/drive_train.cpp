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

    // Both wheels go the same way
    leftDriveDirection = (distanceMM >= 0) ? robotConfig::FORWARD : robotConfig::REVERSE;
    rightDriveDirection = leftDriveDirection;
    
    targetSpeed = abs(speed);

    leftMotor.drive(leftMotor.mapSpeedToDutyCycle(targetSpeed), leftDriveDirection);
    rightMotor.drive(rightMotor.mapSpeedToDutyCycle(targetSpeed), rightDriveDirection);
    
    state = DrivingStraight;
}

void Drivetrain::turn(float degrees, float speed) {
    // 1. Calculate the distance each wheel must travel along the turning circle
    float trackWidth = 202.5; // (mm) Consider moving this to robotConfig in config.h!
    float turningCircumference = PI * trackWidth;
    
    // Distance = Full Circumference * (Requested Degrees / 360 Degrees)
    float distanceMM = turningCircumference * (abs(degrees) / 360.0f);

    // 2. Convert that physical distance into encoder ticks
    float revolutions = distanceMM / robotConfig::WHEEL_1_CIRCUMFERENCE;
    long ticksToMove = revolutions * robotConfig::PULSES_REV;

    // 3. Record starting positions
    leftStartEncoder = leftMotor.encoder.getCount();
    rightStartEncoder = rightMotor.encoder.getCount();

    // 4. Assign directions and targets based on turn direction
    // Assuming positive degrees = Clockwise Turn (Left goes forward, Right goes reverse)
    if (degrees >= 0) {
        leftDriveDirection = robotConfig::FORWARD;
        rightDriveDirection = robotConfig::REVERSE;
        
        leftTargetEncoder = leftStartEncoder + ticksToMove;
        rightTargetEncoder = rightStartEncoder - ticksToMove;
    } else {
        // Negative degrees = Counter-Clockwise Turn
        leftDriveDirection = robotConfig::REVERSE;
        rightDriveDirection = robotConfig::FORWARD;
        
        leftTargetEncoder = leftStartEncoder - ticksToMove;
        rightTargetEncoder = rightStartEncoder + ticksToMove;
    }

    // 5. Kick off the movement
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
    // STATE 1: Actively Driving or Turning (with PD Sync)
    // ---------------------------------------------------------
    if (state == DrivingStraight || state == Turning) {
        
        int leftBasePWM = leftMotor.mapSpeedToDutyCycle(targetSpeed);
        int rightBasePWM = rightMotor.mapSpeedToDutyCycle(targetSpeed);

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

        // Check if we hit targets based on each wheel's independent direction
        bool leftDone = (leftDriveDirection == robotConfig::FORWARD) ? (leftCurrent >= leftTargetEncoder) : (leftCurrent <= leftTargetEncoder);
        bool rightDone = (rightDriveDirection == robotConfig::FORWARD) ? (rightCurrent >= rightTargetEncoder) : (rightCurrent <= rightTargetEncoder);

        // If EITHER wheel finishes, cut power to both immediately to prevent spinning, then correct overshoot
        if (leftDone || rightDone) {
            leftMotor.drive(0, robotConfig::STOPPED);
            rightMotor.drive(0, robotConfig::STOPPED);
            
            // Move to Braking phase to calculate and correct the overshoot
            state = Braking;
        }
    }
    
    // ---------------------------------------------------------
    // STATE 2: One-Time Overshoot Correction
    // ---------------------------------------------------------
    else if (state == Braking) {
        // Calculate exactly how many ticks each wheel overshot by
        long leftOvershoot = leftCurrent - leftTargetEncoder;
        long rightOvershoot = rightCurrent - rightTargetEncoder;

        bool leftNeedsCorrection = abs(leftOvershoot) > 2;
        bool rightNeedsCorrection = abs(rightOvershoot) > 2;

        if (leftNeedsCorrection || rightNeedsCorrection) {
            
            // Run correction at a very low, safe speed
            int leftCorrectionDuty = leftMotor.mapSpeedToDutyCycle(0.05f);
            int rightCorrectionDuty = rightMotor.mapSpeedToDutyCycle(0.05f);
            
            // Apply correction to left wheel if needed
            if (leftNeedsCorrection) {
                // If overshoot is positive, we need to go reverse. If negative, go forward.
                leftDriveDirection = (leftOvershoot > 0) ? robotConfig::REVERSE : robotConfig::FORWARD;
                leftMotor.drive(leftCorrectionDuty, leftDriveDirection);
            } else {
                leftMotor.drive(0, robotConfig::STOPPED);
            }

            // Apply correction to right wheel if needed
            if (rightNeedsCorrection) {
                rightDriveDirection = (rightOvershoot > 0) ? robotConfig::REVERSE : robotConfig::FORWARD;
                rightMotor.drive(rightCorrectionDuty, rightDriveDirection);
            } else {
                rightMotor.drive(0, robotConfig::STOPPED);
            }
            
            // Reset start encoders so the PD sync loop doesn't freak out during the tiny correction push
            leftStartEncoder = leftCurrent;
            rightStartEncoder = rightCurrent;
            targetSpeed = 0.05f; 
            
            // Loop back to Driving state to finish these last few correction ticks cleanly using the same target
            state = DrivingStraight; 
            
        } else {
            // If both wheels are within the 2-tick tolerance buffer, we are perfectly on target!
            leftMotor.drive(0, robotConfig::STOPPED);
            rightMotor.drive(0, robotConfig::STOPPED);
            state = Idle;
        }
    }
}