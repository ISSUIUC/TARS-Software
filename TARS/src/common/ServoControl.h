#pragma once

// TODO why is this in common

#include <PWMServo.h>

#define A 9.8
#define B 2.05
#define C -154
#define D 4.1

class ServoControl {
   public:
    ServoControl(PWMServo* servo, int min, int max);
    void servoActuation(float length);

    int min_angle;
    int max_angle;

   private:
    PWMServo* servo_;
    int roundOffAngle(float value);
};