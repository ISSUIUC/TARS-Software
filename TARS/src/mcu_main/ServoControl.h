#pragma once

// TODO why is this in common

#include "mcu_main/debug.h"

#ifndef ENABLE_SILSIM_MODE
#include "PWMServo.h"
#endif

class ServoControl {
   public:
#ifdef ENABLE_SILSIM_MODE
    ServoControl();
#else
    ServoControl(PWMServo* servo);
#endif
    void servoActuation(float length);

    int min_angle = 37;
    int max_angle = 105;

   private:
#ifndef ENABLE_SILSIM_MODE
    PWMServo* servo_;
#endif
    int roundOffAngle(float value);
};