#pragma once
struct Motor {
public:
    int PWM_pin1;
    int PWM_pin2;
    int encoderPin1;
    int encoderPin2;
    bool isOn = false;
    bool isForward = true;
    float distance = 0.0f;
    float rotations = 0.0f;
    
    volatile long encoderCount = 0; 
    unsigned long lastTime = 0;
    long lastCount = 0;
    float lastKnownSpeed = 0.0; 
    
    Motor(int input_pin1, int input_pin2, int inputEncoderPin1, int inputEncoderPin2);
   //negative speed = backwards 
   void drive(float speed);
   void drive_distance(float distance, float speed = 1.0f);
   void one_turn(void);
   void begin();
   float get_velocity();
   static void isrWrapper(void* arg);
};
