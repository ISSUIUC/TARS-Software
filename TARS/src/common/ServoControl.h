#pragma once

// TODO why is this in common

#include <PWMServo.h>

class ServoControl {
   public:
    ServoControl(PWMServo* servo);
    void servoActuation(float length);

   private:
    PWMServo* servo_;
    void roundOffAngle(float& value);
};