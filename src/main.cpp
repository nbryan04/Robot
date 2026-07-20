#include "config.h"
#include "motor.h"
#include "drive_train.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define I2C_SDA 6
#define I2C_SCL 5

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void OledSetup(){
    bool success = Wire.begin(I2C_SDA, I2C_SCL); 

    // Displays Adafruit logo by default
    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
    display_handler.display();
    delay(2000);
 
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("System Ready");
    display_handler.display();
}

// Pin Definitions
const int PWM_PIN_FORWARD_LEFT = 11;
const int PWM_PIN_REVERSE_LEFT = 10;
const int PWM_PIN_FORWARD_RIGHT = 12;
const int PWM_PIN_REVERSE_RIGHT = 13;
const int ENCODER_PIN1_RIGHT = 4;
const int ENCODER_PIN2_RIGHT = 3;
const int ENCODER_PIN1_LEFT = 1;
const int ENCODER_PIN2_LEFT = 2;
int PWM_FREQ = 2000; // Hz
int PWM_RESOLUTION = 10;

// Motor and Drivetrain Instantiation
Motor motorLeft(PWM_PIN_FORWARD_LEFT, PWM_PIN_REVERSE_LEFT, ENCODER_PIN1_LEFT, ENCODER_PIN2_LEFT, 
                robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR1_POLARITY, robotConfig::leftSpeedToDuty);
                
Motor motorRight(PWM_PIN_FORWARD_RIGHT, PWM_PIN_REVERSE_RIGHT, ENCODER_PIN1_RIGHT, ENCODER_PIN2_RIGHT, 
                 robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR2_POLARITY, robotConfig::rightSpeedToDuty);

Drivetrain drivetrain(motorLeft, motorRight);

// Timer for non-blocking OLED updates
unsigned long lastDisplayTime = 0;

// Define the steps of our specific movement sequence
enum SequenceState {
    SEQ_IDLE,
    SEQ_TURN_OUT,
    SEQ_WAIT_TURN_OUT,
    SEQ_DRIVE_FWD,
    SEQ_WAIT_FWD,
    SEQ_DRIVE_REV,
    SEQ_WAIT_REV,
    SEQ_TURN_IN,
    SEQ_WAIT_TURN_IN,
    SEQ_DONE
};

SequenceState currentSequence = SEQ_IDLE;

void updateMovementSequence() {
    switch (currentSequence) {
        case SEQ_IDLE:
        case SEQ_DONE:
            // Do nothing if the sequence hasn't started or is finished
            break;

        case SEQ_TURN_OUT:
            // Turn 45 degrees clockwise at 0.2 m/s
            drivetrain.turn(45.0, 0.2); 
            currentSequence = SEQ_WAIT_TURN_OUT;
            break;

        case SEQ_WAIT_TURN_OUT:
            // Wait until the drivetrain's internal state machine returns to Idle
            if (drivetrain.state == Drivetrain::Idle) {
                currentSequence = SEQ_DRIVE_FWD;
            }
            break;

        case SEQ_DRIVE_FWD:
            // Drive forward 50mm (5cm) at 0.2 m/s
            drivetrain.driveStraight(50.0, 0.2);
            currentSequence = SEQ_WAIT_FWD;
            break;

        case SEQ_WAIT_FWD:
            if (drivetrain.state == Drivetrain::Idle) {
                currentSequence = SEQ_DRIVE_REV;
            }
            break;

        case SEQ_DRIVE_REV:
            // Drive backward 50mm at 0.2 m/s
            drivetrain.driveStraight(-50.0, 0.2);
            currentSequence = SEQ_WAIT_REV;
            break;

        case SEQ_WAIT_REV:
            if (drivetrain.state == Drivetrain::Idle) {
                currentSequence = SEQ_TURN_IN;
            }
            break;

        case SEQ_TURN_IN:
            // Turn -45 degrees (counter-clockwise) to un-rotate
            drivetrain.turn(-45.0, 0.2);
            currentSequence = SEQ_WAIT_TURN_IN;
            break;

        case SEQ_WAIT_TURN_IN:
            if (drivetrain.state == Drivetrain::Idle) {
                currentSequence = SEQ_DONE; // Sequence complete!
            }
            break;
    }
}

void setup() {
    motorLeft.begin();
    motorRight.begin();
    
    // Kick off the sequence
    currentSequence = SEQ_TURN_OUT;
}

void loop() {
    // 1. Keep the individual motor speed calculations running
    motorLeft.speed();
    motorRight.speed();
    
    // 2. Keep the drivetrain PID and deceleration math running
    drivetrain.update();

    // 3. Keep our new sequence manager running to feed commands to the drivetrain
    updateMovementSequence();

    // 4. Do other things! Check sensors, update your OLED display, move the claw, etc.
    // The ESP32 is completely free to run code here while the robot dances.
}
/*
long ticksToMove = 0; // Global variable to hold the number of ticks to move

void setup() {
    Serial.begin(115200);
    
    motorLeft.begin();
    motorRight.begin();
    OledSetup();
    
    delay(1000); // Brief pause before starting

    // Command the robot to drive 1000mm forward at 0.2 m/s
    chassis.turn(-360, 0.15); // Turn 360 degrees at 0.2 m/s

    float trackWidth = 208; // (mm) 
    float turningCircumference = PI * trackWidth;
    
    float distanceMM = turningCircumference * (abs(360) / 360.0f);

    float revolutions = distanceMM / robotConfig::WHEEL_1_CIRCUMFERENCE;
    ticksToMove = revolutions * robotConfig::PULSES_REV;
}

void loop() {
    // 1. Run the high-speed control loop continuously
    chassis.update();

    // 2. Non-blocking OLED update every 100ms (10Hz)
    unsigned long currentMillis = millis();
    if (currentMillis - lastDisplayTime >= 100) {
        lastDisplayTime = currentMillis;
        
        display_handler.clearDisplay();
        display_handler.setTextSize(1);
        display_handler.setTextColor(SSD1306_WHITE);
        display_handler.setCursor(0,0);
        
        display_handler.print("Goal Ticks: ");
        display_handler.println(ticksToMove);

        // Print Current State (0=Idle, 1=Driving, 2=Turning, 3=Braking)
        display_handler.print("State: ");
        display_handler.println(chassis.state);
        
        // Print Live Speeds
        display_handler.print("L Spd: ");
        display_handler.println(motorLeft.speed(), 3); 
        display_handler.print("R Spd: ");
        display_handler.println(motorRight.speed(), 3);
        
        // Print Current Encoder Ticks for debugging targets
        display_handler.print("L Ticks: ");
        display_handler.println(motorLeft.encoder.getCount());
        display_handler.print("R Ticks: ");
        display_handler.println(motorRight.encoder.getCount());
        
        display_handler.display();
    }
}*/