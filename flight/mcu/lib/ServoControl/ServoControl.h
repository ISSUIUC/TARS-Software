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
    /**
     * @brief actuats servos based on State Estimation.
     * runs during servo thread
     */
    void servoActuation(float length_one);

   private:
    PWMServo* servo_;

    /**
     * @brief ensures angle written to servo is between 0 and 180
     *
     * @param takes an angle as a float
     */
    void roundOffAngle(float& value);
};

#endif