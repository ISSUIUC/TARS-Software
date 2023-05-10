#pragma once

// TODO why is this in common

#include <PWMServo.h>

class ServoControl {
   public:
    ServoControl(PWMServo* servo);
    void servoActuation(float length);

    int min_angle = 37;
    int max_angle = 105;

   private:
    PWMServo* servo_;
    int roundOffAngle(float value);
};