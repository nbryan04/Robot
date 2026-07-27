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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// OLED (SSD1306 over I2C: SDA = 6, SCL = 5, address 0x3C)
// ---------------------------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 6
#define OLED_SCL 5
#define OLED_RESET -1
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


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
const int IMU_SDA_PIN       = 5;   // MPU-6050 shares the I2C bus (SDA)
const int IMU_SCL_PIN       = 6;   // MPU-6050 shares the I2C bus (SCL)
const int LF_LEFT_PIN       = 7;
const int LF_MID_PIN        = 8;
const int LF_RIGHT_PIN      = 9;

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
LineFollower lineFollower(LF_LEFT_PIN, LF_MID_PIN, LF_RIGHT_PIN);

Mission mission(drivetrain, ultrasonic, camera, claw, metalDetector, tiltSensor, lineFollower);

void oledSetup() {
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(400000);   // fast I2C so screen writes disturb the loop less
    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0, 0);
    display_handler.println("System Ready");
    display_handler.display();
}

// Show the metal detector's frequency shift (and context) while scanning a rock.
void showMetalShift() {
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0, 0);
    display_handler.println("Metal scan");
    display_handler.print("Shift: ");
    display_handler.print(metalDetector.getShift(), 1);
    display_handler.println(" Hz");
    display_handler.print("Cur:  ");
    display_handler.println(metalDetector.getCurrentFrequency(), 1);
    display_handler.print("Base: ");
    display_handler.println(metalDetector.getBaseFrequency(), 1);
    display_handler.print("Metal: ");
    display_handler.println(metalDetector.isMetalDetected() ? "YES" : "no");
    display_handler.display();
}

// Show the ultrasonic distance (filtered + raw) for debugging the sweep/approach.
void showUltrasonic() {
    display_handler.clearDisplay();
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setTextSize(1);
    display_handler.setCursor(0, 0);
    display_handler.println("Ultrasonic (cm)");
    display_handler.setTextSize(2);
    display_handler.setCursor(0, 18);
    display_handler.println(ultrasonic.filteredDistanceCm, 1);
    display_handler.setTextSize(1);
    display_handler.setCursor(0, 48);
    display_handler.print("raw ");
    display_handler.println(ultrasonic.currentDistanceCm, 1);
    display_handler.display();
}

// Show the most recent sweep edge (start/end) and how many degrees into the
// sweep arc it was detected. Called from loop() whenever a new edge fires.
void showEdgeEvent() {
    display_handler.clearDisplay();
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setTextSize(2);
    display_handler.setCursor(0, 0);
    if (mission.lastEdgeEvent == Ultrasonic::START_EDGE) {
        display_handler.println("START");
    } else if (mission.lastEdgeEvent == Ultrasonic::END_EDGE) {
        display_handler.println("END");
    } else {
        display_handler.println("--");
    }
    display_handler.println("edge");
    display_handler.setTextSize(1);
    display_handler.setCursor(0, 50);
    display_handler.print("d = ");
    display_handler.print(mission.lastEdgeDeltaDeg, 1);
    display_handler.println(" deg");
    display_handler.display();
}

// Show the finished sweep's result: the start/end edge angles and the distance
// the robot thinks the rock is at. Drawn once per sweep and left on screen.
void showSweepResult() {
    display_handler.clearDisplay();
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setTextSize(1);
    display_handler.setCursor(0, 0);
    display_handler.println(mission.sweepFound ? "ROCK FOUND" : "NO ROCK");

    display_handler.setCursor(0, 18);
    display_handler.print("Start: ");
    display_handler.print(mission.sweepStartAngle, 1);
    display_handler.println(" deg");

    display_handler.setCursor(0, 32);
    display_handler.print("End:   ");
    display_handler.print(mission.sweepEndAngle, 1);
    display_handler.println(" deg");

    display_handler.setCursor(0, 46);
    display_handler.print("Dist:  ");
    display_handler.print(mission.sweepDistanceCm, 1);
    display_handler.println(" cm");

    display_handler.display();
}

void setup() {
    // oledSetup();  // OLED off
    motorLeft.begin();
    motorRight.begin();
    ultrasonic.begin();
    camera.begin();
    claw.begin();
    metalDetector.begin();
    tiltSensor.begin();
    lineFollower.begin();

    // Brief pause before the robot starts moving.
    delay(2000);

    // Sweep + centring disabled for now (line-following ramp focus): hop between
    // clusters without the ultrasonic search, and skip the post-centre realign.
    mission.enableRockSearch = false;  // no FIND_ROCK sweep / travel / centre
    mission.enableTeletubbySweep = true;   // 2s camera scan once aligned on the rock
    mission.enableMetalScan = true;    // run the claw lower/scan/grab at each rock
    mission.testMetalOnRock = 0;       // 0 = use the real detector
    mission.aimMode = Mission::AIM_EDGE_MIDPOINT;  // aim at the midpoint of the two edges
    mission.returnAfterCentre = false;        // no realignment (nothing to undo)

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

    // High-level mission state machine.
    mission.update();
}
