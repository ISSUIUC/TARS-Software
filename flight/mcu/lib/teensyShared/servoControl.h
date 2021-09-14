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
    FSM_State *rocketStatePointer;

    sensorDataStruct_t *lowgSensorDataPointer;

    datalogger_THD *lowgDataloggerTHDVarsPointer;
};

void servoTickFunction(pointers *, PWMServo *, PWMServo *);

void round_off_angle(int &value);

#endif
