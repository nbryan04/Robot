#include "config.h"
#include "motor.h"
#include "drive_train.h"
#include "ultrasonic.h"
#include "camera.h"
#include "claw.h"
#include "metal_detector.h"
#include "tilt_sensor.h"
#include "LineFollower.h"
#include "mission.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
// Drivetrain (known).
const int PWM_PIN_FORWARD_LEFT  = 11;
const int PWM_PIN_REVERSE_LEFT  = 10;
const int PWM_PIN_FORWARD_RIGHT = 12;
const int PWM_PIN_REVERSE_RIGHT = 13;
const int ENCODER_PIN1_RIGHT = 4;
const int ENCODER_PIN2_RIGHT = 3;
const int ENCODER_PIN1_LEFT  = 1;
const int ENCODER_PIN2_LEFT  = 2;

// Peripheral pins: TEMPORARY placeholders, fill in once wired.
const int ULTRA_TRIG_PIN   = -1;
const int ULTRA_ECHO_PIN   = -1;
const int CAMERA_IN_PIN     = -1;  // ESP -> camera (request a detection)
const int CAMERA_OUT_PIN    = -1;  // camera -> ESP (detection result)
const int CLAW_HAND_PIN     = 16;
const int CLAW_ARM_PIN      = 15;
const int METAL_PIN         = 17;
const int TILT_PIN          = -1;
const int LF_LEFT_PIN       = -1;
const int LF_MID_PIN        = -1;
const int LF_RIGHT_PIN      = -1;

// ---------------------------------------------------------------------------
// Object instantiation
// ---------------------------------------------------------------------------
Motor motorLeft(PWM_PIN_FORWARD_LEFT, PWM_PIN_REVERSE_LEFT, ENCODER_PIN1_LEFT, ENCODER_PIN2_LEFT,
                robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR1_POLARITY, robotConfig::leftSpeedToDuty);

Motor motorRight(PWM_PIN_FORWARD_RIGHT, PWM_PIN_REVERSE_RIGHT, ENCODER_PIN1_RIGHT, ENCODER_PIN2_RIGHT,
                 robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR2_POLARITY, robotConfig::rightSpeedToDuty);

Drivetrain   drivetrain(motorLeft, motorRight);
Ultrasonic   ultrasonic(ULTRA_TRIG_PIN, ULTRA_ECHO_PIN);
Camera       camera(CAMERA_IN_PIN, CAMERA_OUT_PIN);
Claw         claw(CLAW_HAND_PIN, CLAW_ARM_PIN);
MetalDetector metalDetector(METAL_PIN);
TiltSensor   tiltSensor(TILT_PIN);
LineFollower lineFollower(LF_LEFT_PIN, LF_MID_PIN, LF_RIGHT_PIN);

Mission mission(drivetrain, ultrasonic, camera, claw, metalDetector, tiltSensor, lineFollower);

void setup() {
    Serial.begin(115200);

    motorLeft.begin();
    motorRight.begin();
    // ultrasonic.begin();  // disabled: unused in nav-only test, pins are -1
    camera.begin();
    claw.begin();
    metalDetector.begin();
    tiltSensor.begin();
    lineFollower.begin();

    // Brief pause before the robot starts moving.
    delay(2000);

    // Claw test: drive the hop path and run the claw lower/scan/grab sequence at
    // each rock. Ultrasonic isn't wired, so skip the search/centre. The metal
    // detector isn't real yet, so fake "metal" on rock 3 to test the grab path.
    mission.enableRockSearch = false;
    mission.enableTeletubbySweep = false;
    mission.enableMetalScan = true;
    mission.testMetalOnRock = 4;   // grab on rock 1; rocks 2,3,4 read as decoys

    mission.begin();
}

void loop() {
    // Low-level control loops.
    motorLeft.speed();
    motorRight.speed();
    drivetrain.update();
    claw.update();   // advances the staged claw actions

    // ultrasonic.update();  // disabled: unused in nav-only test, pins are -1

    // High-level mission state machine.
    mission.update();
}
