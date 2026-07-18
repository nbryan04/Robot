#include "config.h"
#include "motor.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include "claw.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define I2C_SDA 6
#define I2C_SCL 5


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void OledSetup(){
    bool success = Wire.begin(I2C_SDA, I2C_SCL); // sets the I2C pins to the specified values

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);
 
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0); // set the cursor start location
  display_handler.println("Hello World");// Displays "Hello world!" on the screen
  display_handler.display();
}

const int METAL_DETECTOR_PIN = 17;
const int CLAW_PIN = 16;
const int ARM_PIN = 15;

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

int leftSpeedToDuty(float speed) {
    int dutyCycle = 5665 * std::pow(speed, 2) - 542 * speed + 480;
    return std::max(0, std::min(dutyCycle, 1023)); // Ensure duty cycle is within 0-1023 range
}

int rightSpeedToDuty(float speed) {
    int dutyCycle = 7616 * std::pow(speed, 2) - 1016 * speed + 525;
    return std::max(0, std::min(dutyCycle, 1023)); // Ensure duty cycle is within 0-1023 range
}

Motor motorLeft(PWM_PIN_FORWARD_LEFT, PWM_PIN_REVERSE_LEFT, ENCODER_PIN1_LEFT, ENCODER_PIN2_LEFT, robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR1_POLARITY, leftSpeedToDuty);
Motor motorRight(PWM_PIN_FORWARD_RIGHT, PWM_PIN_REVERSE_RIGHT, ENCODER_PIN1_RIGHT, ENCODER_PIN2_RIGHT, robotConfig::WHEEL_1_DIAMETER, robotConfig::MOTOR2_POLARITY, rightSpeedToDuty);
Claw claw(CLAW_PIN, ARM_PIN);
void setup() {
    motorLeft.begin();
    motorRight.begin();
    claw.begin();
    OledSetup();
}


void loop() {
    claw.setAngle(ARM_PIN, robotConfig::ARM_DOWN_ANGLE);
    delay(1000);
    claw.setAngle(ARM_PIN, 45);
    delay(1000);
}

