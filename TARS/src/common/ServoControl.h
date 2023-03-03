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
    float A = 9.8;
    float B = 2.05;
    float C = -154;
    float D = 4.1;
};

#endif