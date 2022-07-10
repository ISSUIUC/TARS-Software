#ifndef SERVO_H
#define SERVO_H

#include <PWMServo.h>

#include "acShared.h"

/**
 * @brief A struct containing pointers to objects needed for the servo_THD.
 *
 */

class ServoControl {
   public:
    ServoControl(PWMServo* servo);
    // ServoControl& operator==(const ServoControl&) = default;
    void servoActuation(float length);

   private:
    PWMServo* servo_;
    void roundOffAngle(float& value);
};

#endif