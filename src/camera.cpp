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
    // Clear any stale/late reply from a previous trigger BEFORE sending, so this
    // scan only accepts a fresh answer to the trigger we're about to send.
    Communications::resetHasNewMessage();
    if (Communications::send_message(data)) {
	Serial.println("cameraTrigger sent successfully"); //debugging statement
        long currentTime = millis();
        while (millis() - currentTime < 3000) {
            if (Communications::hasNewMessage()) {
		bool found = Communications::teletubbyFound();
		// Consume this reply so newMessage doesn't stay latched -- otherwise the
		// next scan sees it already true and returns this stale result instead
		// of waiting for a fresh answer.
		Communications::resetHasNewMessage();
		Serial.println("Message received from camera"); //debugging statements
		Serial.print("Teletubby found:");
		Serial.println(found);
                return found;
            }
        }
    }
    return false;
}
