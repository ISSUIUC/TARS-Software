/**
 * @file        sensors.h
 * @authors     Jake Hawkins
 *              Anshuk Chigullapalli
 *              Ayberk Yaraneri
 *              Magilan Sendhil
 *
 * @brief Globals defined for each sensor
 *
 */

#pragma once

#include "mcu_main/sensors/BarometerSensor.h"
#include "mcu_main/sensors/GPSSensor.h"
#include "mcu_main/sensors/HighGSensor.h"
#include "mcu_main/sensors/LowGSensor.h"
#include "mcu_main/sensors/OrientationSensor.h"
#include "mcu_main/sensors/VoltageSensor.h"
#include "mcu_main/sensors/GasSensor.h"
#include "mcu_main/sensors/MagnetometerSensor.h"

extern HighGSensor highG;
extern LowGSensor lowG;
extern GPSSensor gps;
extern BarometerSensor barometer;
extern VoltageSensor voltage;
extern OrientationSensor orientation;
extern GasSensor gas;
extern MagnetometerSensor magnetometer;
