#ifndef SERVO_H
#define SERVO_H

#include <PWMServo.h>

class ServoControl {
   public:
    ServoControl(PWMServo* servo);
    void servoActuation(float length);

   private:
    PWMServo* servo_;
    void roundOffAngle(float& value);
};

#endif