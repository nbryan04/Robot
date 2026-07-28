#include <Arduino.h>

#include "camera.h"
#include "communications.hpp"

Camera::Camera(int inputPin, int outputPin) {
    this->inputPin = inputPin;
    this->outputPin = outputPin;
}

void Camera::begin() {
    pinMode(inputPin, OUTPUT);    // ESP drives this line into the camera
    pinMode(outputPin, INPUT);    // camera drives this line back to the ESP
    digitalWrite(inputPin, LOW);  // idle: not requesting a detection
    Communications::init();       // start WiFi
}

bool Camera::checkForTeletubby() {
    // Placeholder: pulse the request line, then read the camera's answer.
    // TODO: replace with the real handshake/protocol once the camera is wired.
    /*digitalWrite(inputPin, HIGH);
    bool detected = digitalRead(outputPin) == HIGH;
    digitalWrite(inputPin, LOW); */
    Communications::cameraTrigger data;
    data.triggerState = true;
    if (Communications::send_message(data)) {
	Serial.println("cameraTrigger sent successfully"); //debugging statement
        long currentTime = millis();
        while (millis() - currentTime < 3000) {
            if (Communications::hasNewMessage()) {
		Serial.println("Message received from camera"); //debugging statements
		Serial.print("Teletubby found:");
		Serial.println(Communications::teletubbyFound());
                return Communications::teletubbyFound();
            }
        }
    }
    return false;
}
