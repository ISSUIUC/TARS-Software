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

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
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
    // get read timestamp
    systime_t timeStamp_GPS = chVTGetSystemTime();

    // get the data with a 20 millisecond timeout
    bool has_data = pointer_struct->GPSPointer->getPVT(20);

    if (!has_data) {
        return;
    }

    // Log lat, long, alt, posLock
    // all gps input is in 10^-7 degrees
    float latitude = pointer_struct->GPSPointer->getLatitude();
    float longitude = pointer_struct->GPSPointer->getLongitude();
    // adjust to floating point coordinates
    latitude /= 10000000;
    longitude /= 10000000;
    // altitude input is in mm
    float altitude = pointer_struct->GPSPointer->getAltitude();
    // fixtype 3 means that we have a 3d position fix
    uint32_t fix_type = pointer_struct->GPSPointer->getFixType();
    bool posLock = (fix_type == 3);

    uint32_t SIV_count = pointer_struct->GPSPointer->getSIV();

    // Lock gps mutex
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);

    pointer_struct->sensorDataPointer->gps_data.timeStamp_GPS = timeStamp_GPS;
    pointer_struct->sensorDataPointer->gps_data.latitude = latitude;
    pointer_struct->sensorDataPointer->gps_data.longitude = longitude;
    pointer_struct->sensorDataPointer->gps_data.altitude = altitude;
    pointer_struct->sensorDataPointer->gps_data.posLock = posLock;
    pointer_struct->sensorDataPointer->gps_data.fix_type = fix_type;
    pointer_struct->sensorDataPointer->gps_data.siv_count = SIV_count;

    pointer_struct->dataloggerTHDVarsPointer.gpsFifo.push(
        pointer_struct->sensorDataPointer->gps_data);

    //! Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);

    // Toggle the LED to show if the gps has position lock
    if (posLock == true) {
        digitalWrite(LED_ORANGE, HIGH);
    } else {
        digitalWrite(LED_ORANGE, LOW);
    }

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
        pointer_struct->barometerPointer->getPressure() *
        0.01;  // Converting both of them into correct unit (in mbar)
    pointer_struct->sensorDataPointer->barometer_data.temperature =
        pointer_struct->barometerPointer->getTemperature() *
        0.01;  // Converting both of them into correct unit (in degC)

    // Compute altitude from pres and temp based on Hypsometric equation
    // h = RTln(p0/p)/g, R = specific gas constant; T in K, g for grav. accel.;
    // P for pressure P0 = 1013.25 mbar at sea level; R = 8.314 J/Kmol; g
    // = 9.8m/s^2; Avg. Mole. Mass of air = 0.029kg/mol Some of these parameter
    // may need to change when going into higher altitude, which would need more
    // complex algorithm.

    // pointer_struct->sensorDataPointer->barometer_data.altitude =
    // log(1013.25/pointer_struct->sensorDataPointer->barometer_data.pressure)*8.314*
    // (pointer_struct->sensorDataPointer->barometer_data.temperature+273.15)/9.8/0.029;

    // A version with no divide operation to enable faster computation, probably
    pointer_struct->sensorDataPointer->barometer_data.altitude =
        -log(pointer_struct->sensorDataPointer->barometer_data.pressure *
             0.000987) *
        (pointer_struct->sensorDataPointer->barometer_data.temperature +
         273.15) *
        29.254;

    pointer_struct->dataloggerTHDVarsPointer.barometerFifo.push(
        pointer_struct->sensorDataPointer->barometer_data);
    //! Unlocking &dataMutex for barometer
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer);

#ifdef BAROMETER_DEBUG
    Serial.println("------------- BAROMETER THREAD ---------------");
    Serial.print(pointer_struct->sensorDataPointer->barometer_data.pressure);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->barometer_data.temperature);
    Serial.print(", ");
    Serial.print(pointer_struct->sensorDataPointer->barometer_data.altitude);
#endif
}

#endif
