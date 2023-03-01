/**
 * @file ServoControl.cpp
 *
 * @brief An interface between flight software and our servo
 * library.
 *
 * Provides functionality to write length values
 * directly to the servos instead of raw angles, and helps
 * keep calculated angles within reasonable input margins
 */

#ifndef SERVO_CPP
#define SERVO_CPP

#include "ServoControl.h"

#include <cmath>

ServoControl::ServoControl(PWMServo* servo, int min, int max) {
    servo_ = servo;
    min_angle = min;
    max_angle = max;
}

/**
 * @brief A function to keep the value sent to the servo between 0 and 130
 * degrees.
 *
 * @param value The angle value determined by the control algorithm.
 */
void ServoControl::roundOffAngle(float& value) {
    // Min Extension Angle Value
    if (value > max_angle) {
        value = max_angle;
    }
    // Max Extension Angle Value
    if (value < min_angle) {
        value = min_angle;
    }

    value = std::round(value);
}

/**
 * @brief Takes the length of the flap extension and converts to angles for the
 * servo.
 *
 * @param length Desired flap extension
 */
void ServoControl::servoActuation(float length) {
    // The angle is found through utilizing a fft and mapping extension/angle
    // values to a sine function. len (mm), pass in ang (rad)

    // if (length < 0) length = 0;
    // if (length > 0.018) length = 0.018;
    float angle = ((180 / 3.1415) * asin((length * 1000 + 2) / 20) + 30) / 0.69;

    // Maps the length to an angle based on calibration
    // float angle = -0.035 + 1.09 * pow(10, 3) * length +
    //               2.98 * pow(10, -4) * pow(length, 2) -
    //               1.24 * pow(10, -6) * pow(length, 3);
    roundOffAngle(angle);

    servo_->write(angle);

    // 130 is max
#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif