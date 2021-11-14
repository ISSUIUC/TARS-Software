#include "ActiveControl.h"

#include "PWMServo.h"
#include "ServoControl.h"
#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

ActiveControl::ActiveControl(struct pointers* pointer_struct, PWMServo* ccw,
                             PWMServo* cw)
    : activeControlServos(ccw, cw) {
    gy = &pointer_struct->sensorDataPointer->lowG_data.gy;
    current_state =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    mutex_lowG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG;

    // Flaps go in and out upon initializing for testing purposes
    activeControlServos.servoActuation(0, 0);
    chThdSleepMilliseconds(1000);
    activeControlServos.servoActuation(1, 1);
    chThdSleepMilliseconds(1000);
    activeControlServos.servoActuation(0, 0);
}

void ActiveControl::acTickFunction() {
    chMtxLock(mutex_lowG_);  // Locking only for gy because we use local
                             // variables for everything else
    float e = omega_goal + *gy;
    chMtxUnlock(mutex_lowG_);

    if (true) {
        e_sum += e * .006;
    }
    float dedt = e - e_prev;
    Eigen::Matrix<float, 2, 1> u = (k_p * e) + (k_i * e_sum) + (k_d * dedt);
    float l1 = u(0, 0);
    float l2 = u(1, 0);
    float l1_cmd = 0;
    float l2_cmd = 0;

    if (l1 != 0) {
        int sign = ((l1 - l1_prev) / dt) / abs((l1 - l1_prev) / dt);
        l1_cmd = l1 + sign * min(abs((l1 - l1_prev) / dt), du_max);
    }
    if (l2 != 0) {
        int sign = ((l2 - l2_prev) / dt) / abs((l2 - l2_prev) / dt);
        l2_cmd = l2 + sign * min(abs((l2 - l2_prev) / dt), du_max);
    }

    // low pass filter
    if (l1_cmd < 0.002794) {
        l1_cmd = 0;
    }
    if (l2_cmd < 0.002794) {
        l2_cmd = 0;
    }

    if (ActiveControl_ON()) {
        activeControlServos.servoActuation(l1_cmd, l2_cmd);
    } else {
        activeControlServos.servoActuation(0, 0);
    }

    e_prev = e;
    l1_prev = l1_cmd;
    l2_prev = l2_cmd;
}

bool ActiveControl::ActiveControl_ON() {
    bool active_control_on = false;
    switch (*current_state) {
        case STATE_INIT:
            active_control_on = false;
            break;
        case STATE_IDLE:
            active_control_on = false;
            break;
        case STATE_LAUNCH_DETECT:
            active_control_on = false;
            break;
        case STATE_BOOST:
            active_control_on = false;
            break;
        case STATE_COAST:
            active_control_on = true;
            break;
        case STATE_APOGEE_DETECT:
            active_control_on = false;
            break;
        default:
            active_control_on = false;
            break;
    }
    return active_control_on;
}