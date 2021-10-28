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

<<<<<<< HEAD
#include "dataLog.h"

    == == == =
#include "KX134-1211.h"       //High-G IMU Library
#include "MS5611.h"           //Barometer Library
#include "SparkFunLSM9DS1.h"  //Low-G IMU Library
#include "ZOEM8Q0.hpp"        //GPS Library
#include "dataLog.h"

                 /**
 * @brief A struct containing pointers needed by the lowgIMU_THD
 *
 */
    struct lowg_PNTR {
    LSM9DS1 *lowGimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

/**
 * @brief A struct containing pointers needed by the highgIMU_THD
 *
 */
struct highg_PNTR {
    KX134 *highGimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

/**
 * @brief A struct containing pointers needed by the gps_THD
 *
 */
struct gps_PNTR {
    ZOEM8Q0 *GPSimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

/**
 * @brief A struct containing pointers needed by the barometer_THD
 *
 */
struct barometer_PNTR {
    MS5611 *barometerPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

>>>>>>> master
void lowGimuTickFunction(pointers *);

void highGimuTickFunction(pointers *);

void gpsTickFunction(pointers *);

void barometerTickFunction(pointers *);

#endif
