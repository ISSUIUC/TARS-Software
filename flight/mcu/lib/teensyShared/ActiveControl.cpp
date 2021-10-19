#include "ActiveControl.h"

#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

ActiveControl::ActiveControl(struct pointers* pointer_struct) {
    gx = &pointer_struct->sensorDataPointer->lowG_data.gx;
    current_state = &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
}

void ActiveControl::acTickFunction() {
    float e = omega_goal - *gx;
    if (true) {
        e_sum += e *.006;
    }
    float dedt = e - e_prev;
    Eigen::Matrix<float, 2, 1> u = (k_p * e) + (k_i * e_sum) + (k_d * dedt);
    float l1 = u(0,0);
    float l2 = u(0,1);
    e_prev = e;
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