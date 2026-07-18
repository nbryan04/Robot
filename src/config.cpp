#include "config.h"
#include <Arduino.h>

namespace robotConfig {

// Define mapping for the LEFT motor
int leftSpeedToDuty(float speed) {
    // Safety check to allow the motor to fully stop
    if (speed <= 0.01f) return 0; 
    
    // Derived from your linear region (PWM 400 to 600)
    int calculatedPWM = (1311.5 * speed) + 342;
    
    // Cap it at max duty just to be safe
    return constrain(calculatedPWM, 0, MAX_DUTY);
}

// Define mapping for the RIGHT motor
int rightSpeedToDuty(float speed) {
    if (speed <= 0.01f) return 0;
    
    // Applying the ~1.075x multiple for the right side
    int calculatedPWM = (1410.0 * speed) + 368;
    
    return constrain(calculatedPWM, 0, MAX_DUTY);
}

}