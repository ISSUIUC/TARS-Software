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

#include "dataLog.h"

void lowGimuTickFunction(pointers *);

void highGimuTickFunction(pointers *);

void gpsTickFunction(pointers *);

void barometerTickFunction(pointers *);

#endif
