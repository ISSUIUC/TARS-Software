#ifndef SERVO_H
#define SERVO_H

#include <PWMServo.h>

#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

const float radius = 0.0127; //pitch radius of gear

/**
 * @brief A struct containing pointers to objects needed for the servo_THD.
 *
 */
struct servo_PNTR {
    FSM_State* rocketStatePointer;

    sensorDataStruct_t* lowgSensorDataPointer;

    datalogger_THD* lowgDataloggerTHDVarsPointer;
};

class ServoControl {
   public:
    ServoControl(PWMServo* servo_cw,
                 PWMServo* servo_ccw);
    // ServoControl& operator==(const ServoControl&) = default;
    /**
     * @brief actuats servos based on State Estimation.
     * runs during servo thread
     */
    void servoActuation(float length_one, float length_two);

   private:
    PWMServo* servo_cw_;
    PWMServo* servo_ccw_;

    /**
     * @brief ensures angle written to servo is between 0 and 180
     *
     * @param takes an angle as a float
     */
    void roundOffAngle(float& value);
};

#endif