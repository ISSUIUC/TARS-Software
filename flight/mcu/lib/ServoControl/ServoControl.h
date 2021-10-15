#ifndef SERVO_H
#define SERVO_H

#include <PWMServo.h>

#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

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
    ServoControl(struct pointers* pointer_struct, PWMServo* servo_cw,
                 PWMServo* servo_ccw);
    /**
     * @brief actuats servos based on State Estimation. 
     * runs during servo thread
     */
    void servoActuation(float length_one, float length_two);

   private:
    FSM_State* currState_;
    PWMServo* servo_cw_;
    PWMServo* servo_ccw_;
    mutex_t* mutex_RS_;
    mutex_t* mutex_lowG_;
    float* gz_;

    /**
     * @brief ensures angle written to servo is between 0 and 180
     *  
     * @param takes an angle as a float
     */
    void roundOffAngle(float& value);
};

#endif