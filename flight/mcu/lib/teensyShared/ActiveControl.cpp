#include "ActiveControl.h"

#include "rocketFSM.h"
#include "thresholds.h"

Controller::Controller(struct pointers* pointer_struct, PWMServo* controller_servo)
    : activeControlServos(controller_servo) {
    controller_servo_ = controller_servo;
    stateData_ = &pointer_struct->sensorDataPointer->state_data;
    current_state =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    uint32_t* ac_coast_timer = &pointer_struct->rocketTimers.coast_timer;
    b_alt = &pointer_struct->sensorDataPointer->barometer_data.altitude;
    dataMutex_barometer_ =
        &pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer;

    controller_servo_->write(180);
    chThdSleepMilliseconds(1000);
    controller_servo_->write(15);
    chThdSleepMilliseconds(1000);

    setLaunchPadElevation();
}

void Controller::ctrlTickFunction(pointers* pointer_struct) {
    // chMtxLock(dataMutex_state_);
    array<float, 2> init = {stateData_->state_x, stateData_->state_vx};
    // chMtxUnlock(dataMutex_state_);
    float apogee_est = rk4_.sim_apogee(init, 0.3)[0];

    stateData_->state_apo = apogee_est;

    float u = kp * (apogee_est - apogee_des_msl);

    float min = abs(u - prev_u) / dt;

    if (du_max < min) {
        min = du_max;
    }

    float sign = 1;

    if (u - prev_u < 0) {
        sign = -1;
    }
    u = u + sign * min * dt;
    prev_u = u;

    // Set flap extension limits
    if (u < min_extension) {
        u = min_extension;
    } else if (u > max_extension) {
        u = max_extension;
    }

    if (ActiveControl_ON()) {
        activeControlServos.servoActuation(u);
        pointer_struct->sensorDataPointer->flap_data.l1 = u;

    } else {
        if (pointer_struct->sensorDataPointer->rocketState_data.rocketState ==
            STATE_APOGEE) {
            activeControlServos.servoActuation(0);
            pointer_struct->sensorDataPointer->flap_data.l1 = 0;
        } else {
            activeControlServos.servoActuation(15);
            pointer_struct->sensorDataPointer->flap_data.l1 = 15;
        }
    }
}

bool Controller::ActiveControl_ON() {

    bool active_control_on = true;
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
            // This adds a delay to the coast state so that we don't deploy flaps too quickly
            if (*ac_coast_timer > coast_ac_delay_thresh) {
                active_control_on = true;
            }
            break;
        case STATE_APOGEE_DETECT:\
            active_control_on = false;
            break;
        default:
            active_control_on = false;
            break;
    }
    return active_control_on;
}

void Controller::setLaunchPadElevation() {
    float sum = 0;
    for (int i = 0; i < 30; i++) {
        chMtxLock(dataMutex_barometer_);
        sum += *b_alt;
        chMtxUnlock(dataMutex_barometer_);
        chThdSleepMilliseconds(100);
    }
    launch_pad_alt = sum / 30;
    apogee_des_msl = apogee_des_agl + launch_pad_alt;
}