/**
 * @file        hilsim.cpp
 * @authors     Connor Flynn
 *              Anshuk Chigullapalli
 *              Karnap Patel
 *
 * @brief       Sensor (Low-G, High-G, GPS, barometer, voltage sensor) emulations 
 *              with data pulled from SILSIM over serial
 * 
 * This file includes the HILSIM class which contains fake sensor functions that pull
 * data from SILSIM ove serial instead of actual sensor data
 */

#include <algorithm>
#include <iostream>


#include "hilsim.h"


void hilsim_data_parser() {
    if (Serial.available() > 0) {
    // read the incoming byte:
        incomingByte = Serial.read();

    // say what you got:
        Serial.print("I received: ");
        Serial.println(incomingByte, DEC);
  }

}

class HILSIM{
    void hilsim_parser() {
        // Use Connor's HILSIM parsing here
    }

    void lsm_readAccel(LSM9DS1* lsm) {

        // set accel values in the lsm class based on silsim data
        lsm->ax = scaled.x();
        lsm->ay = scaled.y();
        lsm->az = scaled.z();
    }

    void lsm_readGyro(LSM9DS1* lsm) {
        // set gyro values in the lsm class based on silsim data
        lsm->gx = scaled.x();
        lsm->gy = scaled.y();
        lsm->gz = scaled.z();
    }

    void lsm_readMag(LSM9DS1* lsm) {
        // set magnetometer values in the lsm class based on silsim data
        lsm->mx = scaled.x();
        lsm->my = scaled.y();
        lsm->mz = scaled.z();
    }

    outputData highg_getAccelData() {
        // data comes from HILSIM serial

    }

    bool gps_getPVT(SFE_UBLOX_GNSS* gps, uint16_t maxWait) {
        
        // data is the stuff from serial

        double BigLatDegrees =
            data.x() / 111036.53;  // converted meters to 'rough' degrees @ 40.1164
                                   // degrees latitude
        double BigLongDegrees =
            data.y() / 85269.13;  // converted meters to 'rough' degrees @ 40.1164
                                  // degrees latitude
        double FatHeight = data.z() * 1000;  // converted meters to millimeters
        gps->_Longitude = BigLongDegrees * 10E+7;
        gps->_Latitude = BigLatDegrees * 10E+7;
        gps->_Altitude = FatHeight;

        gps->_isFresh = true;
        gps->_isFreshAltitude = true;
        gps->_isFreshLatitude = true;
        gps->_isFreshLongitude = true;
        return true;
    }

    int32_t gps_getLatitude(SFE_UBLOX_GNSS* gps, uint16_t maxWait) {
        if (!gps->_isFreshLatitude) {
            gps_getPVT(maxWait);
        }
        gps->_isFreshLatitude = false;
        gps->_isFresh = false;
        return gps->_Latitude;
    }
    int32_t gps_getLongitude(SFE_UBLOX_GNSS* gps, uint16_t maxWait) {
        if (!gps->_isFreshLongitude) {
            gps_getPVT(maxWait);
        }
        gps->_isFreshLongitude = false;
        gps->_isFresh = false;
        return gps->_Longitude;
    }
    int32_t gps_getAltitude(SFE_UBLOX_GNSS* gps, uint16_t maxWait) {
        if (!gps->_isFreshAltitude) {
           gps_getPVT(maxWait);
        }
        gps->_isFreshAltitude = false;
        gps->_isFresh = false;
        return gps->_Altitude;
    }

    int barometer_read(MS5611* barometer, uint8_t bits) {
        // get tempKelvins from the HILSIM data buffer
        double tempCelsius = tempKelvins - 273.15;  // first convert to celsius
        barometer->_temperature =
            tempCelsius *
            100;  // finally convert celsius to hundreths of degrees celsius


        // get pressure data from the HILISM data buffer
        barometer->_pressure = pressure_from_hilsi,;
        return 0;
    }

}
