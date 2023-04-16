#pragma once

#include "mcu_main/debug.h"

#ifdef ENABLE_SILSIM_MODE
#include "mcu_main/ISS_SILSIM/src/Sensors/Sensor.h"

extern EmulatedMagnetometerSensor* emulatedMagnetometer;
extern GyroscopeSensor* emulatedGyro;
extern EmulatedGPSSensor* emulatedGPS;
extern Barometer* emulatedMS;
extern Thermometer* emulatedThermometer;
extern Accelerometer* emulatedKX;

#endif
