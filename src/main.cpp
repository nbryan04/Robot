#include "config.h"
#include "motor.h"
#include "drive_train.h"
#include "ultrasonic.h"
#include "camera.h"
#include "claw.h"
#include "metal_detector.h"
#include "tilt_sensor.h"
#include "LineFollower.h"
#include "ir_sensor.h"
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
const int ULTRA_TRIG_PIN   = 40;
const int ULTRA_ECHO_PIN   = 21;
const int CAMERA_IN_PIN     = -1;  // ESP -> camera (request a detection)
const int CAMERA_OUT_PIN    = -1;  // camera -> ESP (detection result)
const int CLAW_HAND_PIN     = 16;
const int CLAW_ARM_PIN      = 15;
const int METAL_PIN         = 17;
const int IMU_SDA_PIN       = 41;   // MPU-6050 shares the I2C bus (SDA)
const int IMU_SCL_PIN       = 42;   // MPU-6050 shares the I2C bus (SCL)
const int LF_LEFT_PIN       = 7;
const int LF_MID_PIN        = 8;
const int LF_RIGHT_PIN      = 9;

// Competition-surface select. The two surfaces differ ONLY in the solar-panel
// removal sequence; this pin (read once at boot) picks which parameter set the
// mission uses. A switch feeds 3V3 into the pin; INPUT_PULLDOWN holds it LOW when
// the switch is open. Switch closed/3V3/HIGH = surface 1, open/0V/LOW = surface 2.
// (GPIO 6 is free now that the OLED is gone.)
const int SURFACE_SELECT_PIN = 6;

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
TiltSensor   tiltSensor(IMU_SDA_PIN, IMU_SCL_PIN);
LineFollower lineFollower;   // logic only; ADC channels are owned by irSensor
IR_Sensor    irSensor(robotConfig::IR_ADC_PIN, robotConfig::IR_SELECT_PIN,
                      LF_LEFT_PIN, LF_MID_PIN, LF_RIGHT_PIN);

Mission mission(drivetrain, ultrasonic, camera, claw, metalDetector, tiltSensor, lineFollower, irSensor);

void setup() {
    Serial.begin(115200);   // USB CDC: sweep-test debug output
    motorLeft.begin();
    motorRight.begin();
    ultrasonic.begin();
    camera.begin();
    claw.begin();
    metalDetector.begin();
    tiltSensor.begin();
    lineFollower.begin();
    // Guarded: IR_Sensor::begin() aborts on an unset ADC pin, so only start it
    // once IR_ADC_PIN is wired in config.h.
    if (robotConfig::IR_ADC_PIN >= 0) irSensor.begin();

    // Brief pause before the robot starts moving.
    delay(2000);

    // Sweep + centring disabled for now (line-following ramp focus): hop between
    // clusters without the ultrasonic search, and skip the post-centre realign.
    mission.enableRockSearch = true;   // full sweep -> travel -> centre
    // Per-rock sweep gate (only used in COLLECT). Example: sweep only rocks 4,5,6;
    // rocks 1-3 hop straight into the scan. Indices are 0-based (rock1..rock6).
    mission.sweepOnRock[0] = false;  // rock 1
    mission.sweepOnRock[1] = false;  // rock 2
    mission.sweepOnRock[2] = false;   // rock 3
    mission.sweepOnRock[3] = false;   // rock 4
    mission.sweepOnRock[4] = true;   // rock 5
    mission.sweepOnRock[5] = true;   // rock 6
    mission.enableTeletubbySweep = true;   // ~5s camera scan once aligned on the rock
    mission.enableMetalScan = true;    // run the claw lower/scan/grab at each rock
    mission.testMetalOnRock = 0;       // 0 = use the real detector
    mission.aimMode = Mission::AIM_EDGE_MIDPOINT;  // aim at the midpoint of the two edges
    mission.returnAfterCentre = true;         // realign: reverse the travel/centre distance back to the arrival pose

    // Surface select (read once at boot): switch feeds 3V3, pulldown holds LOW when
    // open. 3V3/HIGH = surface 1, 0V/LOW = surface 2. Only the solar-panel removal
    // parameters differ between the two surfaces.
    pinMode(SURFACE_SELECT_PIN, INPUT_PULLDOWN);
    mission.panelSurface = (digitalRead(SURFACE_SELECT_PIN) == LOW) ? 1 : 0;
    Serial.printf("[SURFACE] pin%d %s -> surface %d\n", SURFACE_SELECT_PIN,
                  mission.panelSurface == 1 ? "LOW" : "HIGH", mission.panelSurface + 1);

    // TEST: panel phase. Jump straight to CREST -> FIND_LINE -> FOLLOW_LINE so the
    // IR beacon detection can be tuned. Set back to COLLECT for a real run.
    mission.phase = Mission::COLLECT;

    mission.begin();
}

void loop() {
    // Low-level control loops.
    motorLeft.speed();
    motorRight.speed();
    drivetrain.update();
    claw.update();          // advances the staged claw actions

    ultrasonic.update();    // ping + filter + edge detection for the sweep
    metalDetector.update(); // updates the running frequency / shift
    tiltSensor.update();    // reads the MPU-6050, updates the ramp latch
    irSensor.update();      // drains the IR DMA buffer + Goertzel (no-op until searching)

    // Panel-test debug: print the IR beacon strength vs its per-tone threshold so
    // the 1kHz/10kHz values can be calibrated. Throttled to 100ms while searching.
    if (irSensor.sampling()) {
        static unsigned long lastIrPrint = 0;
        if (millis() - lastIrPrint >= 100) {
            lastIrPrint = millis();
            Serial.printf("[IR] f=%.0fHz  mag=%.4f  thr=%.4f  %s | LF L=%d M=%d R=%d (thr=%d)\n",
                          irSensor.targetFreq(), irSensor.magnitude(),
                          irSensor.threshold(),
                          irSensor.detected() ? "DETECTED" : "searching",
                          irSensor.lfLeftRaw(), irSensor.lfMidRaw(), irSensor.lfRightRaw(),
                          robotConfig::LF_THRESHOLD);
        }
    }

    // Tilt debug: an event line the moment the ramp latch flips, plus a live angle
    // readout (throttled to 200ms) so rampOnAngle/rampOffAngle can be calibrated.
    // Only prints once the MPU-6050 is actually detected on the bus.
    if (tiltSensor.isPresent()) {
        static bool lastOnRamp = false;
        bool onRamp = tiltSensor.isOnRamp();
        if (onRamp != lastOnRamp) {
            Serial.printf("[TILT] ramp %s  angle=%.1f deg\n",
                          onRamp ? "TRIGGERED" : "cleared", tiltSensor.getTiltAngle());
            lastOnRamp = onRamp;
        }
        static unsigned long lastTiltPrint = 0;
        if (millis() - lastTiltPrint >= 200) {
            lastTiltPrint = millis();
            Serial.printf("[TILT] angle=%.1f deg  onRamp=%d\n",
                          tiltSensor.getTiltAngle(), onRamp ? 1 : 0);
        }
    }

    // High-level mission state machine.
    mission.update();
}
