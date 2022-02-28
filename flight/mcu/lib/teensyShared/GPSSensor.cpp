#include "GPSSensor.h"

#include <ChRt.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

void GPSSensor::readReadings() { GNSS->getPVT(20); }

float GPSSensor::getLatitude() {
    latitude = GNSS->getLatitude();
    return latitude;
}

float GPSSensor::getLongtitude() {
    longtitude = GNSS->getLongitude();
    return longtitude;
}

float GPSSensor::getAltitude() {
    altitude = GNSS->getAltitude();
    return altitude;
}

uint32_t GPSSensor::getFixType() {
    fix_type = GNSS->getFixType();
    return fix_type;
}

bool GPSSensor::getPosLock() { return (fix_type == 3); }

uint32_t GPSSensor::getSIVCount() {
    SIV_count = GNSS->getSIV();
    return SIV_count;
}
