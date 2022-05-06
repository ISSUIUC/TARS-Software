#include "ActiveControl.h"
#include <iostream>
#include <string>
#include <fstream>

Controller::Controller(struct pointers* pointer_struct, PWMServo* twisty_boi): activeControlServos(twisty_boi) {
    twisty_boi_ = twisty_boi;
    stateData_ = &pointer_struct->sensorDataPointer->state_data;
    current_state =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    // dataMutex_state_ = &pointer_struct->dataloggerTHDVarsPointer->dataMutex_state;
}

void Controller::ctrlTickFunction() {
    // chMtxLock(dataMutex_state_);
    array<float, 2> init = {stateData_->state_x, stateData_->state_vx};
    // chMtxUnlock(dataMutex_state_);
    float apogee_est = rk4_.sim_apogee(init, 0.1)[0];

    float u = kp*(apogee_est - apogee_des);

    float min = abs(u - prev_u)/dt;

    if (du_max < min) {
        min = du_max;
    }

    float sign = 1;

    if (u - prev_u < 0) {
        sign = -1;
    }
    u = u + sign*min*dt;
    prev_u = u;


    //Set flap extension limits
    if (u < min_extension) {
        u = min_extension;
    } else if (u > max_extension) {
        u = max_extension;
    }

    // std::cout << "Controller Flap Extension: " << u << std::endl;


    if (ActiveControl_ON()) {
        activeControlServos.servoActuation(u);
    } else {
        activeControlServos.servoActuation(0);
    }

}

bool Controller::ActiveControl_ON() {

    // For some reason, this only sees IDLE through launch.

    bool active_control_on = true;
    switch (*current_state) {
        case STATE_INIT:
            // std::cout << "INIT" << std::endl;
            active_control_on = false;
            break;
        case STATE_IDLE:
            // std::cout << "IDLE" << std::endl;
            active_control_on = false;
            break;
        case STATE_LAUNCH_DETECT:
            // std::cout << "Launch Detect" << std::endl;
            active_control_on = false;
            break;
        case STATE_BOOST:
            // std::cout << "BOOST" << std::endl;
            active_control_on = false;
            break;
        case STATE_COAST:
            // std::cout << "COAST" << std::endl;
            active_control_on = true;
            break;
        case STATE_APOGEE_DETECT:
            // std::cout << "Apogee" << std::endl;
            active_control_on = false;
            break;
        default:
            active_control_on = false;
            break;
    }
    return active_control_on;
}