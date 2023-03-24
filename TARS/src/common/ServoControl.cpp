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

#define A 9.8
#define B 2.05
#define C -154
#define D 4.1

ServoControl::ServoControl(PWMServo* servo) : servo_(servo) {}

/**
 * @brief A function to bound the desired servo angle between the limits
 * defined by the constructor.
 *
 * @param value The angle value determined by the control algorithm.
 * @return The bounded angle value as an integer.
 */
int ServoControl::roundOffAngle(float value) {
    // Min Extension Angle Value
    if (value > max_angle) {
        value = max_angle;
    } else if (value < min_angle) {
        value = min_angle;
    }

    return std::round(value);
}

/**
 * @brief Takes the length of the flap extension and converts to angles for the
 * servo.
 *
 * @param length Desired flap extension in mm.
 */
void ServoControl::servoActuation(float length) {
    // The angle is found through utilizing a fft and mapping extension/angle
    // values to a sine function. len (mm), pass in ang (rad)

    // if (length < 0) length = 0;
    // if (length > 0.018) length = 0.018;
    // float angle = ((180 / 3.1415) * asin((length * 1000 + 2) / 20) + 30) / 0.69;
    float angle = (((180 / 3.1415) * asin((length - D) / A)) - C) / B;

    // Maps the length to an angle based on calibration
    // float angle = -0.035 + 1.09 * pow(10, 3) * length +
    //               2.98 * pow(10, -4) * pow(length, 2) -
    //               1.24 * pow(10, -6) * pow(length, 3);
    // roundOffAngle(angle);
    int servo_angle = roundOffAngle(angle);

    servo_->write(servo_angle);

    // 130 is max
#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif