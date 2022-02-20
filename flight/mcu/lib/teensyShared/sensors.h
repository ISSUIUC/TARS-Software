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

void lowGimuTickFunction(LSM9DS1* LSM_Pointer, datalogger_THD* THD_Datalog_Buffer, lowGData* lowG_Data);

void highGimuTickFunction(KX134* highG, datalogger_THD* data_log_buffer, highGData* state_buffer);

void gpsTickFunction(SFE_UBLOX_GNSS* GPSPointer, datalogger_THD* dataloggerTHDVarsPointer, gpsData* gps_data);

void barometerTickFunction(pointers *);

#endif
