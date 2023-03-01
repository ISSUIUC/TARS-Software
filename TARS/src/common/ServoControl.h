#pragma once

// TODO why is this in common

#include <PWMServo.h>

class ServoControl {
   public:
    ServoControl(PWMServo* servo, int min, int max);
    void servoActuation(float length);

   private:
    PWMServo* servo_;
    void roundOffAngle(float& value);
    int min_angle;
    int max_angle;
};