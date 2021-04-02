#ifndef SERVO_H
#define SERVO_H

#include "dataLog.h"
#include "acShared.h"



struct servo_PNTR {
    FSM_State *rocketStatePointer;

    sensorDataStruct_t *lowgSensorDataPointer;

    datalogger_THD *lowgDataloggerTHDVarsPointer;
};

#endif