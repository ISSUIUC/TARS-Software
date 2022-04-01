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
#include <cmath>

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
// #include "thresholds.h"
#include "pins.h"
#include "sensors.h"

static int dummy_input_lowG = 0;
static int dummy_input_GPS = 0;

/**
 * @brief Construct a new thd function object to handle data collection from the
 * low-g IMU.
 *
 * @param arg Contains pointers to various objects needed by the low-g IMU.
 *
 */
void lowGimuTickFunction(LSM9DS1* lsm, DataLogBuffer* data_log_buffer,
                         LowGData* lowG_Data) {
    // Reads data from the low g IMU

    // Looping input value from 0 to 2pi over and over 
    if (dummy_input_lowG > 628) {
        dummy_input_lowG = 0;
    } else {
        dummy_input_lowG+=30;
    }

    // Computing sine value
    double sin_value = sin(dummy_input_lowG/100);
    double cos_value = cos(dummy_input_lowG/100);
    double tan_value = tan(dummy_input_lowG/100);


    

    chSysLock();
    lsm->readAccel();
    lsm->readGyro();
    lsm->readMag();
    chSysUnlock();

    // Lock low g mutex
    chMtxLock(&data_log_buffer->dataMutex_lowG);

    // Log timestamp
    lowG_Data->timeStamp_lowG = chVTGetSystemTime();

    // Log acceleration in Gs
    lowG_Data->ax = lsm->calcAccel(lsm->ax);
    lowG_Data->ay = lsm->calcAccel(lsm->ay);
    lowG_Data->az = lsm->calcAccel(lsm->az);  // There was a minus here. We
                                              // don't know why that did that
    // Log rotational speed in degrees per second
    lowG_Data->gx = lsm->calcGyro(lsm->gx);
    lowG_Data->gy = lsm->calcGyro(lsm->gy);
    lowG_Data->gz = lsm->calcGyro(lsm->gz);
    // Log magnetometer data in gauss
    lowG_Data->mx = lsm->calcMag(lsm->mx);
    lowG_Data->my = lsm->calcMag(lsm->my);
    lowG_Data->mz = lsm->calcMag(lsm->mz);
    //! Unlocking &dataMutex for low g

    data_log_buffer->pushLowGFifo(lowG_Data);
    chMtxUnlock(&data_log_buffer->dataMutex_lowG);

#ifdef LOWGIMU_DEBUG
    Serial.println("------------- LOW-G THREAD ---------------");
    Serial.print(lowG_data->ax);
    Serial.print(", ");
    Serial.print(lowG_data->ay);
    Serial.print(", ");
    Serial.print(lowG_data->az);
    Serial.print(", ");
    Serial.print(lowG_data->gx);
    Serial.print(", ");
    Serial.print(lowG_data->gy);
    Serial.print(", ");
    Serial.print(lowG_data->gz);
    Serial.print(", ");
    Serial.print(lowG_data->mx);
    Serial.print(", ");
    Serial.print(lowG_data->my);
    Serial.print(", ");
    Serial.print(lowG_data->mz);
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
void gpsTickFunction(SFE_UBLOX_GNSS* gps, DataLogBuffer* data_log_buffer,
                     GpsData* gps_data) {
    // get read timestamp
    systime_t timeStamp_GPS = chVTGetSystemTime();

    // get the data with a 20 millisecond timeout
    bool has_data = gps->getPVT(20);

    if (!has_data) {
        return;
    }

    // Log lat, long, alt, posLock
    // all gps input is in 10^-7 degrees
    float latitude = gps->getLatitude();
    float longitude = gps->getLongitude();
    // adjust to floating point coordinates
    latitude /= 10000000;
    longitude /= 10000000;
    // altitude input is in mm
    float altitude = gps->getAltitude();
    // fixtype 3 means that we have a 3d position fix
    uint32_t fix_type = gps->getFixType();
    bool posLock = (fix_type == 3);

    uint32_t SIV_count = gps->getSIV();

    mutex_t& gps_mutex = data_log_buffer->dataMutex_GPS;
    // Lock gps mutex
    chMtxLock(&gps_mutex);

    gps_data->timeStamp_GPS = timeStamp_GPS;
    gps_data->latitude = latitude;
    gps_data->longitude = longitude;
    gps_data->altitude = altitude;
    gps_data->posLock = posLock;
    gps_data->fix_type = fix_type;
    gps_data->siv_count = SIV_count;

    data_log_buffer->pushGpsFifo(gps_data);

    //! Unlocking &dataMutex
    chMtxUnlock(&gps_mutex);

    // Toggle the LED to show if the gps has position lock
    if (posLock == true) {
        digitalWrite(LED_ORANGE, HIGH);
    } else {
        digitalWrite(LED_ORANGE, LOW);
    }

#ifdef GPS_DEBUG
    bool position_lock = pointer_struct->gps->get_position_lock();
    if (position_lock) {
        Serial.println("POSITION LOCK!");
        Serial.println("GPS Data: ");
        Serial.print("Latitude: ");
        Serial.println(gps_data->latitude);
        Serial.print("Longitude: ");
        Serial.println(gps_data->longitude);
        Serial.print("Altitude: ");
        Serial.println(gps_data->altitude);
    } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(gps_data->latitude);
        Serial.print("Longitude: ");
        Serial.println(gps_data->longitude);
        Serial.print("Altitude: ");
        Serial.println(gps_data->altitude);
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
void highGimuTickFunction(KX134* highG, DataLogBuffer* data_log_buffer,
                          HighGData* highg_data) {
    // Read data from high g IMU
    chSysLock();
    highG->update_data();
    chSysUnlock();

    // Lock high g mutex
    chMtxLock(&data_log_buffer->dataMutex_highG);

    // Log high g timestamp
    highg_data->timeStamp_highG = chVTGetSystemTime();

    // Log accelerations highg_data
    highg_data->hg_ax = highG->get_x_gforce();
    highg_data->hg_ay = highG->get_y_gforce();
    highg_data->hg_az = highG->get_z_gforce();

    // Unlock high g mutex

    data_log_buffer->pushHighGFifo(highg_data);

    chMtxUnlock(&data_log_buffer->dataMutex_highG);

#ifdef HIGHGIMU_DEBUG
    Serial.println("------------- HIGH-G THREAD ---------------");
    // high g data
    Serial.print(highG_data->hg_ax);
    Serial.print(", ");
    Serial.print(highG_data->hg_ay);
    Serial.print(", ");
    Serial.println(highG_data->hg_az);
#endif
}

/**
 * @brief Construct a new thd function object to handle data collection from the
 * barometer.
 *
 * @param arg Contains pointers to various objects needed by the barometer.
 *
 */
// void barometerTickFunction(pointers *pointer_struct) {
void barometerTickFunction(MS5611* barometer, DataLogBuffer* data_log_buffer,
                           BarometerData* barometer_data) {
    // Reads data from the barometer
    // chSysLock();
    barometer->read(12);
    // chSysUnlock();

    // Lock barometer mutex
    chMtxLock(&data_log_buffer->dataMutex_barometer);

    // Log timestamp
    barometer_data->timeStamp_barometer = chVTGetSystemTime();

    // Log pressure and temperature
    barometer_data->pressure =
        barometer->getPressure() *
        0.01;  // Converting both of them into correct unit (in mbar)
    barometer_data->temperature =
        barometer->getTemperature() *
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
    barometer_data->altitude = -log(barometer_data->pressure * 0.000987) *
                               (barometer_data->temperature + 273.15) * 29.254;

    data_log_buffer->pushBarometerFifo(barometer_data);
    //! Unlocking &dataMutex for barometer
    chMtxUnlock(&data_log_buffer->dataMutex_barometer);

#ifdef BAROMETER_DEBUG
    Serial.println("------------- BAROMETER THREAD ---------------");
    Serial.print(barometer_data->pressure);
    Serial.print(", ");
    Serial.print(barometer_data->temperature);
    Serial.print(", ");
    Serial.print(barometer_data->altitude);
#endif
}

#endif
