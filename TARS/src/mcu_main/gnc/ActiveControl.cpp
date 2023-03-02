/**
 * @file ActiveControl.cpp
 *
 * @brief Contains the C++ implementation of the active controls math,
 * logic to tell controls that it's safe to actuate based on FSM, and
 * functionality to set the launch pad elevation at startup.
 */

#include "mcu_main/gnc/ActiveControl.h"

#include "mcu_main/finite-state-machines/rocketFSM.h"
#include "mcu_main/gnc/kalmanFilter.h"

Controller activeController;

/**
 * @brief Initializes the Servos for either drag or roll control
 * 
 * Initializes a PWMServo object and sets the angle limits for the ServoControl object
*/
Controller::Controller() : activeControlServos(&controller_servo_, 35, 105) {}

void Controller::ctrlTickFunction() {
    chMtxLock(&kalmanFilter.mutex);
    array<float, 2> init = {kalmanFilter.getState().x, kalmanFilter.getState().vx};
    chMtxUnlock(&kalmanFilter.mutex);

    float apogee_est = rk4_.sim_apogee(init, 0.3)[0];

    chMtxLock(&kalmanFilter.mutex);
    kalmanFilter.updateApogee(apogee_est);
    chMtxUnlock(&kalmanFilter.mutex);

    float u = kp * (apogee_est - apogee_des_msl);

    // Limit rate of the servo so that it does not command a large change in a
    // short period of time
    float min = abs(u - prev_u) / dt;

    if (du_max < min) {
        min = du_max;
    }

    // Update servo input with rate limited value
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

    /*
     * When in COAST state, we set the flap extension to whatever the AC
     * algorithm calculates If not in COAST, we keep the servos at 15 degrees.
     * This was experimentally determined to be the position where the flaps
     * were perfectly flush with the airframe. After APOGEE is detected, we set
     * the servos to 0 degrees so that the avionics bay can be removed from the
     * airframe with ease on the ground
     */
    if (ActiveControl_ON()) {
        activeControlServos.servoActuation(u);
        dataLogger.pushFlapsFifo((FlapData){u, chVTGetSystemTime()});
    } else {
        activeControlServos.servoActuation(min_extension);
    }
}

/**
 * @brief Determines whether it's safe for flaps to actuate. Does this
 * based on FSM state
 * @returns boolean depending on whether flaps should actuate or not
 */
bool Controller::ActiveControl_ON() { return getActiveFSM().getFSMState() == FSM_State::STATE_COAST_GNC; }

/**
 * @brief Initializes launchpad elevation through barometer measurement.
 *
 * This method takes a series of barometer measurements on start up and takes
 * the average of them in order to initialize the target altitude to a set value
 * above ground level. Hard coding a launch pad elevation is not a viable
 * solution to this problem as the Kalman filter which is the data input to the
 * controller uses barometric altitude as its reference frame. This is
 * equivalent to determining the barometric pressure at an airport and using it
 * to calibrate an aircraft's onboard altimeter.
 *
 * The function takes an average of 30 measurements, each made 100 ms apart
 */
void Controller::setLaunchPadElevation() {
    float sum = 0;
    for (int i = 0; i < 30; i++) {
        chMtxLock(&barometer.mutex);
        sum += barometer.getAltitude();
        chMtxUnlock(&barometer.mutex);
        chThdSleepMilliseconds(100);
    }
    launch_pad_alt = sum / 30;
    apogee_des_msl = apogee_des_agl + launch_pad_alt;
}

void Controller::init() {
    controller_servo_.attach(AC_SERVO_PIN, 770, 2250);

    /*
     * Startup sequence
     * 30 degrees written to servo since this was
     * experimentally determined to be the position in which
     * the flaps are perfectly flush with the airframe.
     */
    
    controller_servo_.write(activeControlServos.max_angle);
    chThdSleepMilliseconds(1000);
    controller_servo_.write(activeControlServos.max_angle);
    chThdSleepMilliseconds(1000);


    setLaunchPadElevation();
}
