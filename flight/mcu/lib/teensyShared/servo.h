#ifndef SERVO_H
#define SERVO_H

#include "acShared.h"
#include "dataLog.h"

/**
 * @brief A struct containing pointers to objects needed for the servo_THD.
 *
 */
struct servo_PNTR {
    FSM_State *rocketStatePointer;

    sensorDataStruct_t *lowgSensorDataPointer;

    datalogger_THD *lowgDataloggerTHDVarsPointer;
};

void round_off_angle(int &value);
#endif