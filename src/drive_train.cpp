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

    // Both Driving and Turning use the same sync logic, just evaluating their specific targets
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

        if (leftDone || rightDone) {
            // Setup independent braking directions
            leftBrakeDirection = (leftDriveDirection == robotConfig::FORWARD) ? robotConfig::REVERSE : robotConfig::FORWARD;
            rightBrakeDirection = (rightDriveDirection == robotConfig::FORWARD) ? robotConfig::REVERSE : robotConfig::FORWARD;
            
            // Set targets `brakeTicks` away in the new direction
            leftBrakeTarget = leftCurrent + ((leftBrakeDirection == robotConfig::FORWARD) ? brakeTicks : -brakeTicks);
            rightBrakeTarget = rightCurrent + ((rightBrakeDirection == robotConfig::FORWARD) ? brakeTicks : -brakeTicks);
            
            leftMotor.drive(robotConfig::MAX_DUTY / 2, leftBrakeDirection);
            rightMotor.drive(robotConfig::MAX_DUTY / 2, rightBrakeDirection);
            
            state = Braking;
        }
    }
    else if (state == Braking) {
        // Evaluate completion based on the BRAKING direction
        bool leftBrakeDone = (leftBrakeDirection == robotConfig::FORWARD) ? (leftCurrent >= leftBrakeTarget) : (leftCurrent <= leftBrakeTarget);
        bool rightBrakeDone = (rightBrakeDirection == robotConfig::FORWARD) ? (rightCurrent >= rightBrakeTarget) : (rightCurrent <= rightBrakeTarget);

        // Turn off motors individually as they finish their reverse pulse
        if (leftBrakeDone) leftMotor.drive(0, robotConfig::STOPPED);
        if (rightBrakeDone) rightMotor.drive(0, robotConfig::STOPPED);

        if (leftBrakeDone && rightBrakeDone) {
            state = Idle; 
        }
    }
}