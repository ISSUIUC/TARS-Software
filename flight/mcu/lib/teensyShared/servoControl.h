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

class ServoControl {
    public:
        ServoControl(struct pointers* pointer_struct) {
            currState = pointer_struct->sensorDataPointer->rocketState_data.rocketState;
        }

        void servoTickFunction(pointers *, PWMServo *, PWMServo *);

    private:
        FSM_State currState;

        void roundOffAngle(int &value);
}

#endif
