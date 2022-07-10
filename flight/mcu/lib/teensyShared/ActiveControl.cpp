/* ActiveControl.cpp
 * 
 * Contains the C++ implementation of the active controls math, 
 * logic to tell controls that it's safe to actuate based on FSM, and
 * functionality to set the launch pad elevation at startup.
 */ 

#include "ActiveControl.h"

#include "rocketFSM.h"
#include "thresholds.h"

Controller::Controller(struct pointers* pointer_struct,
                       PWMServo* controller_servo)
    : activeControlServos(controller_servo) {
    controller_servo_ = controller_servo;
    stateData_ = &pointer_struct->sensorDataPointer->state_data;
    current_state =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    ac_coast_timer = &pointer_struct->rocketTimers.coast_timer;
    b_alt = &pointer_struct->sensorDataPointer->barometer_data.altitude;
    dataMutex_barometer_ =
        &pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer;

    /* 
     * Startup sequence
     * 15 degrees written to servo since this was
     * experimentally determined to be the position in which
     * the flaps are perfectly flush with the airframe.
     */
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

    /**
     * When in COAST state, we set the flap extension to whatever the AC
     *algorithm calculates If not in COAST, we keep the servos at 15 degrees.
     *This was experimentally determined to be the position where the flaps were
     *perfectly flush with the airframe. After APOGEE is detected, we set the
     *servos to 0 degrees so that the avionics bay can be removed from the
     *airframe with ease on the ground
     **/
    if (ActiveControl_ON()) {
        activeControlServos.servoActuation(u);
        pointer_struct->sensorDataPointer->flap_data.extension = u;

    } else {
        if (pointer_struct->sensorDataPointer->rocketState_data.rocketState ==
            STATE_APOGEE) {
            activeControlServos.servoActuation(0);
            pointer_struct->sensorDataPointer->flap_data.extension = 0;
        } else {
            activeControlServos.servoActuation(15);
            pointer_struct->sensorDataPointer->flap_data.extension = 15;
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
            // This adds a delay to the coast state so that we don't deploy
            // flaps too quickly
            if (*ac_coast_timer > coast_ac_delay_thresh) {
                active_control_on = true;
            }
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

/* This method takes a series of barometer measurements on start up and takes
 * the average of them in order to initialize the target altitude to a set value
 * above ground level. Hard coding a launch pad elevation is not a viable
 * solution to this problem as the Kalman filter which is the data input to the
 * controller uses barometric altitude as its reference frame. This is
 * equivalent to determining the barometric pressure at an airport and using it
 * to calibrate an aircraft's onboard altimeter.
 */
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