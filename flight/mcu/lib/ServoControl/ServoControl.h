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

    void servoTickFunction();

   private:
    FSM_State* currState_;
    PWMServo* servo_cw_;
    PWMServo* servo_ccw_;
    mutex_t* mutex_RS_;
    mutex_t* mutex_lowG_;
    float* gz_;
    void roundOffAngle(float& value);
};

#endif
