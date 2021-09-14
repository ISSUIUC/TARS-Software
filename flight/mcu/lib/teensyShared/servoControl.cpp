#ifndef SERVO_CPP
#define SERVO_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
// TODO remove
#include "KX134-1211.h"       //High-G IMU Library
#include "SparkFunLSM9DS1.h"  //Low-G IMU Library
#include "ZOEM8Q0.hpp"        //GPS Library
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
#include "pins.h"
#include "sensors.h"
#include "servo.h"
#include "thresholds.h"

float flap_drag;
float native_drag;

/**
 * @brief A function to keep the value sent to the servo between 0 and 180
 * degrees.
 *
 * @param value The value determined by the control algorithm.
 */
void round_off_angle(int& value) {
    if (value > 180) {
        value = 180;
    }
    if (value < 0) {
        value = 0;
    }
}

/**
 * @brief Construct a new thd function object to control the servo.
 *
 * @param arg A struct containing pointers to objects needed to run the thread.
 *
 */
void servoTickFunction(pointers* pointer_struct, PWMServo* servo_cw,
                       PWMServo* servo_ccw) {
    int ccw_angle = 90;
    int cw_angle = 90;

    static bool active_control = false;

    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
    FSM_State currentRocketState =
        pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);

    switch (currentRocketState) {
        case STATE_INIT:
            active_control = true;
            break;
        case STATE_IDLE:
            active_control = true;
            break;
        case STATE_LAUNCH_DETECT:
            active_control = true;
            break;
        case STATE_BOOST:
            active_control = false;
            break;
        case STATE_COAST:
            active_control = true;
            break;
        case STATE_APOGEE_DETECT:
            active_control = false;
            break;
        default:
            active_control = false;
            break;
    }
    // turns active control off if not in takeoff/coast sequence
    if (active_control) {
        chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
        cw_angle = pointer_struct->sensorDataPointer->lowG_data.gz;
        ccw_angle = pointer_struct->sensorDataPointer->lowG_data.gz;
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);

    } else {
        // Turns active control off if not in coast state.
        cw_angle = 0;
        ccw_angle = 0;
    }
    round_off_angle(cw_angle);
    round_off_angle(ccw_angle);

    servo_cw->write(cw_angle);
    servo_ccw->write(ccw_angle);

#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif
