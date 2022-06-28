#ifndef SERVO_CPP
#define SERVO_CPP

#include "ServoControl.h"

#include <cmath>

/**
 * @brief A function to keep the value sent to the servo between 0 and 180
 * degrees.
 *
 * @param value The value determined by the control algorithm.
 */
ServoControl::ServoControl(PWMServo* servo) { servo_ = servo; }
// TODO check values for max
void ServoControl::roundOffAngle(float& value) {
    // Min Extension Angle Value
    if (value > 130) {
        value = 130;
    }
    // Max Extension Angle Value
    if (value < 0) {
        value = 0;
    }

    value = std::round(value);
}

/**
 * @brief Takes the length of the flap extension and converts to angles for the
 * servo.
 *
 * @param length_one The length of the flap extension for the counterclockwise
 * flaps.
 * @param length_two The length of the flap extension for the clockwise flaps.
 *
 */
void ServoControl::servoActuation(float length) {
    // The angle is found through utilizing a fft and mapping extension/angle
    // values to a sine function. len (mm), pass in ang (rad)

    if (length < 0) length = 0;
    if (length > 0.018) length = 0.018;

    /* Maps the length to an angle based on calibration */
    float angle = -0.035 + 1.09 * pow(10, 3) * length +
                  2.98 * pow(10, -4) * pow(length, 2) -
                  1.24 * pow(10, -6) * pow(length, 3);
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