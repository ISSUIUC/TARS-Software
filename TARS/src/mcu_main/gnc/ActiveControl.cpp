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
#include "mcu_main/buzzer/buzzer.h"

Controller activeController;

/**
 * @brief Initializes the Servos for either drag or roll control
 * 
 * Initializes a PWMServo object and sets the angle limits for the ServoControl object
*/
Controller::Controller() : activeControlServos(&controller_servo_, 30, 110) {}

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

void buzz_count(int n) {
    for (int i = 0; i < n; i++) {
        tone(15, 200);
        delay(100);
        noTone(15);
        delay(50);
    }
}
void Controller::init() {
    controller_servo_.attach(AC_SERVO_PIN, 770, 2250);

    /*
     * Startup sequence
     * 30 degrees written to servo since this was
     * experimentally determined to be the position in which
     * the flaps are perfectly flush with the airframe.
     */
    
    int angle;
    while (1) {
        for (int i = 3; i <= 11; i++) {
            buzz_count(i);
            if (i == 3) {
                angle = 35;
            } else {
                angle = i * 10;
            }
            controller_servo_.write(i * 10);
            delay(10000);
        }
    }
    // char curr_val = '0';
    // int angle = 30;
    // while (1) {
    //     if (Serial.available()) {
    //         char serial_data = Serial.read();
    //         if (serial_data != 10) {
    //             curr_val = serial_data;
    //         }
    //     }
    //     delay(1000);
    //     switch(curr_val) {
    //         case '1':
    //             angle = 30;
    //             break;
    //         case '2':
    //             angle = 40;
    //             break;
    //         case '3':
    //             angle = 50;
    //             break;
    //         case '4':
    //             angle = 60;
    //             break;
    //         case '5':
    //             angle = 70;
    //             break;
    //         case '6':
    //             angle = 80;
    //             break;
    //         case '7':
    //             angle= 90;
    //             break;
    //         case '8':
    //             angle = 100;
    //             break;
    //         case '9':
    //             angle = 110;
    //             break;
    //     }
    //     controller_servo_.write(angle);
    // }


    setLaunchPadElevation();
}
