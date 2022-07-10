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

#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
#include "pins.h"
#include "thresholds.h"

rocketFSM::rocketFSM(pointers *ptr) { pointer_struct = ptr; }

void rocketFSM::tickFSM() {
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Lock mutexes for data used in switch
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    // get the linear accelration from the lowgimu
    float linear_acceleration =
        pointer_struct->sensorDataPointer->highG_data.hg_az;

    // links to abort for other states
    if (pointer_struct->abort) {
        pointer_struct->sensorDataPointer->rocketState_data.rocketState =
            STATE_ABORT;
        // Serial.println("ABORT");
    }

    switch (pointer_struct->sensorDataPointer->rocketState_data.rocketState) {
        case STATE_ABORT:
            // if true, always stay in abort
            break;

        case STATE_INIT:
            // go to state idle regardless of gps lock
            // Serial.println("INIT");
            pointer_struct->sensorDataPointer->rocketState_data.rocketState =
                STATE_IDLE;

            break;

        case STATE_IDLE:
            // Serial.println("IDLE");
            // If high acceleration is observed in z direction...
            if (linear_acceleration > launch_linear_acceleration_thresh) {
                pointer_struct->rocketTimers.launch_time = chVTGetSystemTime();
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_LAUNCH_DETECT;
            }

            break;

        case STATE_LAUNCH_DETECT:
            // Serial.println("\n LAUNCH DETECTED \n");
            // If the acceleration was too brief, go back to IDLE
            if (linear_acceleration < launch_linear_acceleration_thresh) {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_IDLE;
                break;
            }

            // measure the length of the burn time (for hysteresis)
            pointer_struct->rocketTimers.burn_timer =
                chVTGetSystemTime() - pointer_struct->rocketTimers.launch_time;

            // If the acceleration lasts long enough, boost is detected
            if (TIME_I2MS(pointer_struct->rocketTimers.burn_timer) >
                launch_time_thresh) {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_BOOST;
                // digitalWrite(LED_RED, HIGH);
            }

            break;

        case STATE_BOOST:
            // Serial.println("BOOST");
            Serial.println(linear_acceleration);
            pointer_struct->rocketTimers.burn_timer =
                chVTGetSystemTime() - pointer_struct->rocketTimers.launch_time;
            // If low acceleration in the Z direction...
            if (linear_acceleration < coast_thresh) {
                // Serial.println("Acceleration below thresh");
                pointer_struct->rocketTimers.burnout_time = chVTGetSystemTime();
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_BURNOUT_DETECT;
                break;
            }
            // Keeping rocket in STATE_BOOST if time below a certain threshold
            if (TIME_I2MS(pointer_struct->rocketTimers.burn_timer) <
                burn_time_thresh_ms) {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_BOOST;
            }
            // Forcing rocket to go to STATE_COAST if threshold crossed
            else {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_COAST;
                // Setting burnout time because we don't otherwise
                pointer_struct->rocketTimers.burnout_time = chVTGetSystemTime();
            }

            break;

        case STATE_BURNOUT_DETECT:
            // Serial.println("\n BURNOUT DETECTED \n");
            // If the 0 acceleration was too brief, go back to BOOST
            if (linear_acceleration > coast_thresh) {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_BOOST;
                break;
            }

            // measure the length of the coast time (for hysteresis)
            pointer_struct->rocketTimers.coast_timer =
                chVTGetSystemTime() - pointer_struct->rocketTimers.burnout_time;

            // If the low acceleration lasts long enough, coast is detected
            if (TIME_I2MS(pointer_struct->rocketTimers.coast_timer) >
                coast_time_thresh) {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_COAST;
            }

            break;

        case STATE_COAST:
            // Serial.println("COAST");
            pointer_struct->rocketTimers.coast_timer =
                chVTGetSystemTime() - pointer_struct->rocketTimers.burnout_time;

            if (TIME_I2MS(pointer_struct->rocketTimers.coast_timer) >
                coast_to_apogee_time_thresh) {
                pointer_struct->sensorDataPointer->rocketState_data
                    .rocketState = STATE_APOGEE;
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
