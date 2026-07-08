#pragma once
struct LineFollower {
    int lerror = 0; //last error
    int error = 0;
    int lPin;
    int mPin;
    int rPin;
    LineFollower(int lSensorPin, int mSensorPin, int rSensorPin);
    void begin();
    void update();
};
