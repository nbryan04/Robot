#pragma once
#include <Arduino.h>

// Placeholder for the camera / vision module.
//
// The camera is treated as a separate co-processor that talks to the ESP over
// two digital lines:
//   - inputPin : ESP -> camera. The ESP raises this to ask the camera to run
//                a detection (a "trigger" / enable line).
//   - outputPin: camera -> ESP. The camera raises this when it has found a
//                teletubby (a "result" / flag line).
//
// TODO: real wiring/protocol TBD. For now checkForTeletubby() just samples the
// output line so the rest of the code can be written against this interface.
struct Camera {
    Camera(int inputPin, int outputPin);

    int inputPin;   // ESP -> camera (request a detection)
    int outputPin;  // camera -> ESP (detection result)

    void begin();

    // Ask the camera whether it currently sees a teletubby.
    // Returns true if one is detected.
    bool checkForTeletubby();
};
