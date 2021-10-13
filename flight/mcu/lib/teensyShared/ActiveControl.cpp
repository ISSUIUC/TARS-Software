#include "ActiveControl.h"

#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

ActiveControl::ActiveControl(struct pointers* pointer_struct) {
    gx = &pointer_struct->sensorDataPointer->lowG_data.gx;
}

void ActiveControl::acTickFunction() {
    float e = omega_goal - *gx;
    if (true) {
        e_sum += e *.006;
    }
    float dedt = e - e_prev;
    Matrix<float, 2, 1> u;
    u = (k_p * e) + (k_i * e_sum) + (k_d * dedt);
    float l1 = u(0,0);
    float l2 = u(0,1);
    e_prev = e;
}

// bool ActiveControl::ActiveControl_ON() {
//     switch (currentRocketState) {
//         case STATE_INIT:
//             active_control = false;
//             break;
//         case STATE_IDLE:
//             active_control = false;
//             break;
//         case STATE_LAUNCH_DETECT:
//             active_control = false;
//             break;
//         case STATE_BOOST:
//             active_control = false;
//             break;
//         case STATE_COAST:
//             active_control = true;
//             break;
//         case STATE_APOGEE_DETECT:
//             active_control = false;
//             break;
//         default:
//             active_control = false;
//             break;
//     }
//     return active_control;
// }