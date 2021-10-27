/**
 * @file        sensors.cpp
 * @authors     Jake Hawkins
 *              Anshuk Chigullapalli
 *              Ayberk Yaraneri
 *
 * @brief       Sensor (Low-G, High-G and GPS) function implementations
 *
 * Here contains the pointer structs for three types of sensors (Low-g, High-g
 * and GPS) and the functions to acquire data from the sensors to the pointer
 * structs.
 *
 */

#ifndef SENSORS_CPP
#define SENSORS_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// #include "SparkFunLSM9DS1.h" //Low-G IMU Library
// #include "KX134-1211.h" //High-G IMU Library
// #include "ZOEM8Q0.hpp" //GPS Library
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
// #include "thresholds.h"
#include "pins.h"
#include "sensors.h"

/**
 * @brief Construct a new thd function object to handle data collection from the
 * low-g IMU.
 *
 * @param arg Contains pointers to various objects needed by the low-g IMU.
 *
 */
void lowGimuTickFunction(pointers *pointer_struct) {
    // Reads data from the low g IMU
    chSysLock();
    pointer_struct->lowGimuPointer->readAccel();
    pointer_struct->lowGimuPointer->readGyro();
    pointer_struct->lowGimuPointer->readMag();
    chSysUnlock();

    // Lock low g mutex
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);

    // Log timestamp
    pointer_struct->sensorDataPointer->lowG_data.timeStamp_lowG =
        chVTGetSystemTime();

    // Log acceleration in Gs
    pointer_struct->sensorDataPointer->lowG_data.ax =
        pointer_struct->lowGimuPointer->calcAccel(
            pointer_struct->lowGimuPointer->ax);
    pointer_struct->sensorDataPointer->lowG_data.ay =
        pointer_struct->lowGimuPointer->calcAccel(
            pointer_struct->lowGimuPointer->ay);
    pointer_struct->sensorDataPointer->lowG_data
        .az = pointer_struct->lowGimuPointer->calcAccel(
        pointer_struct->lowGimuPointer->az);  // There was a minus here. We
                                              // don't know why that did that
    // Log rotational speed in degrees per second
    pointer_struct->sensorDataPointer->lowG_data.gx =
        pointer_struct->lowGimuPointer->calcGyro(
            pointer_struct->lowGimuPointer->gx);
    pointer_struct->sensorDataPointer->lowG_data.gy =
        pointer_struct->lowGimuPointer->calcGyro(
            pointer_struct->lowGimuPointer->gy);
    pointer_struct->sensorDataPointer->lowG_data.gz =
        pointer_struct->lowGimuPointer->calcGyro(
            pointer_struct->lowGimuPointer->gz);
    // Log magnetometer data in gauss
    pointer_struct->sensorDataPointer->lowG_data.mx =
        pointer_struct->lowGimuPointer->calcMag(
            pointer_struct->lowGimuPointer->mx);
    pointer_struct->sensorDataPointer->lowG_data.my =
        pointer_struct->lowGimuPointer->calcMag(
            pointer_struct->lowGimuPointer->my);
    pointer_struct->sensorDataPointer->lowG_data.mz =
        pointer_struct->lowGimuPointer->calcMag(
            pointer_struct->lowGimuPointer->mz);
    //! Unlocking &dataMutex for low g

    pointer_struct->dataloggerTHDVarsPointer.lowGFifo.push(
        pointer_struct->sensorDataPointer->lowG_data);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);

#ifdef LOWGIMU_DEBUG
    Serial.println("------------- LOW-G THREAD ---------------");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.ax);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.ay);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.az);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.gx);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.gy);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.gz);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.mx);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.my);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->lowG_data.mz);
    Serial.print(", ");
#endif
}

/**
 * @brief Construct a new thd function object to handle data collection from the
 * GPS.
 *
 * @param arg Contains pointers to various objects needed by the GPS.
 *
 */
void gpsTickFunction(pointers *pointer_struct) {
    // Read data from gps
    chSysLock();
    pointer_struct->GPSPointer->update_data();
    chSysUnlock();

    // Lock gps mutex
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);

    // Log gps timestamp
    pointer_struct->sensorDataPointer->gps_data.timeStamp_GPS =
        chVTGetSystemTime();

    // Have the availability to wait until a lock is aquired with
    // gps.get_position_lock();

    // Log lat, long, alt, posLock
    pointer_struct->sensorDataPointer->gps_data.latitude =
        pointer_struct->GPSPointer->get_latitude();
    pointer_struct->sensorDataPointer->gps_data.longitude =
        pointer_struct->GPSPointer->get_longitude();
    pointer_struct->sensorDataPointer->gps_data.altitude =
        pointer_struct->GPSPointer->get_altitude();
    pointer_struct->sensorDataPointer->gps_data.posLock =
        pointer_struct->GPSPointer->get_position_lock();

    //! Unlocking &dataMutex

    // Toggle the LED to show if the gps has position lock
    if (pointer_struct->sensorDataPointer->gps_data.posLock == true) {
        digitalWrite(LED_ORANGE, HIGH);
    } else {
        digitalWrite(LED_ORANGE, LOW);
    }

    pointer_struct->dataloggerTHDVarsPointer.gpsFifo.push(
        pointer_struct->sensorDataPointer->gps_data);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);

#ifdef GPS_DEBUG
    bool position_lock = pointer_struct->GPSPointer->get_position_lock();
    if (position_lock) {
        Serial.println("POSITION LOCK!");
        Serial.println("GPS Data: ");
        Serial.print("Latitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.latitude);
        Serial.print("Longitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.longitude);
        Serial.print("Altitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.altitude);
    } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.latitude);
        Serial.print("Longitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.longitude);
        Serial.print("Altitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.altitude);
        Serial.println("");
    }
#endif
}

/**
 * @brief Construct a new thd function object to handle data collection from the
 * high-g IMU.
 *
 * @param arg Contains pointers to the various objects needed by the high-g IMU.
 *
 */
void highGimuTickFunction(pointers *pointer_struct) {
    // Read data from high g IMU
    chSysLock();
    pointer_struct->highGimuPointer->update_data();
    chSysUnlock();

    // Lock high g mutex
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    // Log high g timestamp
    pointer_struct->sensorDataPointer->highG_data.timeStamp_highG =
        chVTGetSystemTime();

    // Log accelerations from high g
    pointer_struct->sensorDataPointer->highG_data.hg_ax =
        pointer_struct->highGimuPointer->get_x_gforce();
    pointer_struct->sensorDataPointer->highG_data.hg_ay =
        pointer_struct->highGimuPointer->get_y_gforce();
    pointer_struct->sensorDataPointer->highG_data.hg_az =
        pointer_struct->highGimuPointer->get_z_gforce();

    // Unlock high g mutex

    pointer_struct->dataloggerTHDVarsPointer.highGFifo.push(
        pointer_struct->sensorDataPointer->highG_data);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

#ifdef HIGHGIMU_DEBUG
    Serial.println("------------- HIGH-G THREAD ---------------");
    // high g data
    Serial.print(pointer_struct->sensorDataPointer->highG_data.hg_ax);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->highG_data.hg_ay);
    Serial.print(", ");
    Serial.println(pointer_struct->sensorDataPointer->highG_data.hg_az);
#endif
}

/**
 * @brief Construct a new thd function object to handle data collection from the
 * barometer.
 *
 * @param arg Contains pointers to various objects needed by the barometer.
 *
 */
void barometerTickFunction(pointers *pointer_struct) {
    // Reads data from the barometer
    // chSysLock();
    pointer_struct->barometerPointer->read(12);
    // chSysUnlock();

    // Lock barometer mutex
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer);

    // Log timestamp
    pointer_struct->sensorDataPointer->barometer_data.timeStamp_barometer =
        chVTGetSystemTime();

    // Log pressure and temperature 
    pointer_struct->sensorDataPointer->barometer_data.pressure =
        pointer_struct->barometerPointer->getPressure()*0.01; // Converting both of them into correct unit (Probably millibars)
    pointer_struct->sensorDataPointer->barometer_data.temperature =
        pointer_struct->barometerPointer->getTemperature()*0.01; // Converting both of them into correct unit (Probably degreeC)

    pointer_struct->dataloggerTHDVarsPointer.barometerFifo.push(pointer_struct->sensorDataPointer->barometer_data);
    //! Unlocking &dataMutex for barometer
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer);

#ifdef BAROMETER_DEBUG
    Serial.println("------------- BAROMETER THREAD ---------------");
    Serial.print(pointer_struct->sensorDataPointer->barometer_data.pressure);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->barometer_data.temperature);
#endif
}


#endif
