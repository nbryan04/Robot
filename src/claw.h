#pragma once
struct Claw {
public:
    Claw(int hpin, int apin);
    int hpin;
    int apin;
    void setAngle(int pin, int angle);
    void begin();
    void grabRock();
};
