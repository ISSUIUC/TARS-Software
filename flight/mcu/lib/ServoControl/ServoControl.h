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
ServoControl::ServoControl(PWMServo* servo_cw,
                           PWMServo* servo_ccw) {
    servo_cw_ = servo_cw;
    servo_ccw_ = servo_ccw;
}

// TODO check values for max
void ServoControl::roundOffAngle(float& value) {
    if (value > 126) {
        value = 126;
    }
    if (value < 0) {
        value = 0;
    }
}

/**
 * @brief Takes the length of the flap extension and converts to angles for the servo.
 *
 * @param length_one The length of the flap extension for the counterclockwise flaps.
 * @param length_two The length of the flap extension for the clockwise flaps.
 *
 */
void ServoControl::servoActuation(float length_one, float length_two) {
    //These are correcting factors for finding the angle. We still need to calculate what
    //these values are. These are placeholders for now. m and offset should include the radian to 
    //degree conversion.
    float m = (1 / radius) * (180 / (3.1415)); // degrees / meter
    float offset = 0;

    //This is the actual conversion from the inputted length to the angles desired for the servos.
    float ccw_angle = (length_one * m  + offset);
    float cw_angle = (length_two * m + offset);

    
    roundOffAngle(cw_angle);
    roundOffAngle(ccw_angle);
    // cw_angle = 0;
    // ccw_angle = 0;
    servo_cw_->write(126 - cw_angle);
    servo_ccw_->write(ccw_angle);

#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif