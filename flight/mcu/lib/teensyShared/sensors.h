/**
 * @file        sensors.h
 * @authors     Jake Hawkins
 *              Anshuk Chigullapalli
 *              Ayberk Yaraneri
 *
 * @brief       Sensor (Low-G, High-G and GPS) struct definitions
 *
 * Here contains the pointer structs for three types of sensors (Low-g, High-g
 * and GPS) and the functions to acquire data from the sensors to the pointer
 * structs.
 *
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "HighGSensor.h"
#include "LowGSensor.h"
#include "GPSSensor.h"
#include "BarometerSensor.h"
#include "VoltageSensor.h"

extern HighGSensor highG;
extern LowGSensor lowG;
extern GPSSensor gps;
extern BarometerSensor barometer;
extern VoltageSensor voltage;

//MUTEX_DECL(flaps_mutex);

#endif
