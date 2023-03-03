#pragma once

// TODO why is this in common

#include <PWMServo.h>

class ServoControl {
   public:
    ServoControl(PWMServo* servo, int min, int max);
    void servoActuation(float length);

    int min_angle;
    int max_angle;

   private:
    PWMServo* servo_;
    void roundOffAngle(float& value);
    float A = 9.8;
    float B = 2.05;
    float C = -154;
    float D = 4.1;
};