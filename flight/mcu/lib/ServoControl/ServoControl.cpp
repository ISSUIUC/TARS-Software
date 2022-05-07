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
ServoControl::ServoControl(PWMServo* servo) {
    servo_ = servo;
}
// TODO check values for max
void ServoControl::roundOffAngle(float& value) {
    //Min Extension Angle Value
    if (value > 130) {
        value = 130;
    }
    //Max Extension Angle Value
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
    // The angle is found through utilizing a fft and mapping extension/angle values to 
    // a sine function. len (mm), pass in ang (rad)

    if(length < 0) length = 0;
    if(length > 0.018) length = 0.018;

    float angle = (26+67.309*asin(0.0553*(length*1000)+0.003904));
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