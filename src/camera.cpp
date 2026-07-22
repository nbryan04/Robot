#include "camera.h"
#include <Arduino.h>
#include "communications.hpp"

Camera::Camera(int inputPin, int outputPin) {
    this->inputPin = inputPin;
    this->outputPin = outputPin;
}

void Camera::begin() {
    pinMode(inputPin, OUTPUT);   // ESP drives this line into the camera
    pinMode(outputPin, INPUT);   // camera drives this line back to the ESP
    digitalWrite(inputPin, LOW); // idle: not requesting a detection
    Communications::init(); //start WiFi
}

bool Camera::checkForTeletubby() {

    // Placeholder: pulse the request line, then read the camera's answer.
    // TODO: replace with the real handshake/protocol once the camera is wired.
    /*digitalWrite(inputPin, HIGH);
    bool detected = digitalRead(outputPin) == HIGH;
    digitalWrite(inputPin, LOW); */
    Communications::struct_message data;
    data.triggerState = true;
    Communications::send_message(data);
    return false; //placeholder since we only have 1-way communication. 
}
