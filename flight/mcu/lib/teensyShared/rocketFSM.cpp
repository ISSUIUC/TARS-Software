/**
 * @file        rocketFSM.cpp
 * @authors     Anshuk Chigullapalli
 * 		Ayberk Yaraneri
 * 		Colin Kinsey
 *
 * @brief      The implementation of the finite state machine class that governs
 * state transitions.
 *
 * The rocketFSM class encapsulates the finite state machine that dictates which
 * state the rocket is in throughout the mission. The class implements the logic
 * necessary to reliably transition between states along with hysteresis to
 * avoid premature state transitions.
 *
 * This is a highly critical software module and should be tested throughly in
 * simulation and on hardware targets.:
 *
 */

#include "rocketFSM.h"

#include <Arduino.h>
#include <ChRt.h>
#include <SPI.h>
#include <Wire.h>

// #include "acShared.h"
#include "dataLog.h"
#include "pins.h"
#include "thresholds.h"

rocketFSM::rocketFSM(pointers *ptr) {
    pointer_struct = ptr;
    // get the linear accelration from the lowgimu
    linear_acceleration_ptr_ = &pointer_struct->sensorDataPointer->highG_data.hg_az;
    // storing pointer_struct variables locally for use
    rocket_state_ptr_ = &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    launch_time_ptr_ = &pointer_struct->rocketTimers.launch_time;
    burn_timer_ptr_ = &pointer_struct->rocketTimers.burn_timer;
    burnout_time_ptr_ = &pointer_struct->rocketTimers.burnout_time;
    coast_timer_ptr_ = &pointer_struct->rocketTimers.coast_timer;
}

void rocketFSM::tickFSM() {
    // Lock mutexes for data used in switch
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    // links to abort for other states
    if (pointer_struct->abort) {
        *rocket_state_ptr_ =
            STATE_ABORT; // definitely put rocket state in constructor
        // Serial.println("ABORT");
    }

    switch (*rocket_state_ptr_) {
        case STATE_ABORT:
            // if true, always stay in abort
            break;

        case STATE_INIT:
            // go to state idle regardless of gps lock
            // Serial.println("INIT");
            *rocket_state_ptr_ =
                STATE_IDLE;

            break;

        case STATE_IDLE:
            // Serial.println("IDLE");
            // If high acceleration is observed in z direction...
            if (*linear_acceleration_ptr_ > launch_linear_acceleration_thresh) {
                *launch_time_ptr_ = chVTGetSystemTime();
                *rocket_state_ptr_ = STATE_LAUNCH_DETECT;
            }

            break;

        case STATE_LAUNCH_DETECT:
            // Serial.println("\n LAUNCH DETECTED \n");
            // If the acceleration was too brief, go back to IDLE
            if (*linear_acceleration_ptr_ < launch_linear_acceleration_thresh) {
                *rocket_state_ptr_ = STATE_IDLE;
                break;
            }

            // measure the length of the burn time (for hysteresis)
            *burn_timer_ptr_ = chVTGetSystemTime() - *launch_time_ptr_;

            // If the acceleration lasts long enough, boost is detected
            if (TIME_I2MS(*burn_timer_ptr_) >
                launch_time_thresh) {
                *rocket_state_ptr_ = STATE_BOOST;
                // digitalWrite(LED_RED, HIGH);
            }

            break;

        case STATE_BOOST:
            // Serial.println("BOOST");
            Serial.println(*linear_acceleration_ptr_);
            *burn_timer_ptr_ =
                chVTGetSystemTime() - *launch_time_ptr_;
            // If low acceleration in the Z direction...
            if (*linear_acceleration_ptr_ < coast_thresh) {
                // Serial.println("Acceleration below thresh");
                *burnout_time_ptr_ = chVTGetSystemTime();
                *rocket_state_ptr_ = STATE_BURNOUT_DETECT;
                break;
            }
            // Keeping rocket in STATE_BOOST if time below a certain threshold
            if (TIME_I2MS(*burn_timer_ptr_) <
                burn_time_thresh_ms) {
                *rocket_state_ptr_ = STATE_BOOST;
            }
            // Forcing rocket to go to STATE_COAST if threshold crossed
            else {
                *rocket_state_ptr_ = STATE_COAST;
                // Setting burnout time because we don't otherwise
                *burnout_time_ptr_ = chVTGetSystemTime();
            }

            break;

        case STATE_BURNOUT_DETECT:
            // Serial.println("\n BURNOUT DETECTED \n");
            // If the 0 acceleration was too brief, go back to BOOST
            if (*linear_acceleration_ptr_ > coast_thresh) {
                *rocket_state_ptr_ = STATE_BOOST;
                break;
            }

            // measure the length of the coast time (for hysteresis)
            *coast_timer_ptr_ = chVTGetSystemTime() - *burnout_time_ptr_;

            // If the low acceleration lasts long enough, coast is detected
            if (TIME_I2MS(*coast_timer_ptr_) >
                coast_time_thresh) {
                *rocket_state_ptr_ = STATE_COAST;
            }

            break;

        case STATE_COAST:
            // Serial.println("COAST");
            *coast_timer_ptr_ = chVTGetSystemTime() - *burnout_time_ptr_;

            if (TIME_I2MS(*coast_timer_ptr_) >
                coast_to_apogee_time_thresh) {
                *rocket_state_ptr_ = STATE_APOGEE;
            } else {
                // Serial.println("Still in coast");
            }

            break;
        case STATE_APOGEE:
            // Serial.println("APOGEE");
        default:
            break;
    }
    // Update timestamp for when rocket state was polled
    pointer_struct->sensorDataPointer->rocketState_data.timeStamp_RS =
        chVTGetSystemTime();

    pointer_struct->dataloggerTHDVarsPointer.pushRocketStateFifo(
        &pointer_struct->sensorDataPointer->rocketState_data);

    // Unlock mutexes used during the switch statement
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);
}
