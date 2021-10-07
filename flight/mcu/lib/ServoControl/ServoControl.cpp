#ifndef SERVO_CPP
#define SERVO_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
// TODO remove
#include "KX134-1211.h"  //High-G IMU Library
#include "ServoControl.h"
#include "SparkFunLSM9DS1.h"  //Low-G IMU Library
#include "ZOEM8Q0.hpp"        //GPS Library
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
#include "pins.h"
#include "sensors.h"
#include "thresholds.h"

float flap_drag;
float native_drag;

/**
 * @brief A function to keep the value sent to the servo between 0 and 180
 * degrees.
 *
 * @param value The value determined by the control algorithm.
 */
ServoControl::ServoControl(struct pointers* pointer_struct, PWMServo* servo_cw,
                           PWMServo* servo_ccw) {
    currState_ =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    servo_cw_ = servo_cw;
    servo_ccw_ = servo_ccw;
    mutex_RS_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS;
    mutex_lowG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG;
    gz_ = &pointer_struct->sensorDataPointer->lowG_data.gz;
}
void ServoControl::roundOffAngle(float& value) {
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
void ServoControl::servoTickFunction() {
    float ccw_angle = 90;
    float cw_angle = 90;

    static bool active_control = false;

    chMtxLock(mutex_RS_);
    FSM_State currentRocketState = *currState_;
    chMtxUnlock(mutex_RS_);

    switch (currentRocketState) {
        case STATE_INIT:
            active_control = false;
            break;
        case STATE_IDLE:
            active_control = false;
            break;
        case STATE_LAUNCH_DETECT:
            active_control = false;
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
        chMtxLock(mutex_lowG_);
        cw_angle = *gz_;  // stand-in "implementation"
        ccw_angle = *gz_;
        chMtxUnlock(mutex_lowG_);

    } else {
        // Turns active control off if not in coast state.
        cw_angle = 0;
        ccw_angle = 0;
    }
    roundOffAngle(cw_angle);
    roundOffAngle(ccw_angle);

    servo_cw_->write(cw_angle);
    servo_ccw_->write(ccw_angle);

#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif
