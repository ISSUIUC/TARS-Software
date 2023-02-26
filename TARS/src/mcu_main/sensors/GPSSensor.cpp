#include "mcu_main/sensors/GPSSensor.h"

#include "mcu_main/dataLog.h"
#include "mcu_main/pins.h"

GPSSensor gps;

ErrorCode GPSSensor::init() {
//    SPI1.begin();  // TODO should this line be moved?
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_ORANGE, HIGH);

    if (!GNSS.begin(Wire)) {
     return ErrorCode::CANNOT_CONNECT_ZOEM8Q0;
    }

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_ORANGE, LOW);

    GNSS.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX);  // Set the SPI port to output UBX only
    // (turn off NMEA noise)
    GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save (only) the communications port settings
    // to flash and BBR
    GNSS.setNavigationFrequency(5);  // set sampling rate to 5hz
    return ErrorCode::NO_ERROR;
}

void GPSSensor::update() {
    chMtxLock(&mutex);
    bool succeed = GNSS.getPVT(20);
    if (!succeed) {
        return;
    }

    timeStamp = chVTGetSystemTime();
    latitude = static_cast<float>(GNSS.getLatitude() / 10000000.0);
    longitude = static_cast<float>(GNSS.getLongitude() / 10000000.0);
    altitude = static_cast<float>(GNSS.getAltitude());
    fix_type = GNSS.getFixType();
    pos_lock = fix_type == 3;
    SIV_count = GNSS.getSIV();

    dataLogger.pushGpsFifo((GpsData){latitude, longitude, altitude, SIV_count, fix_type, pos_lock, timeStamp});
    chMtxUnlock(&mutex);
}

float GPSSensor::getLatitude() const { return latitude; }

float GPSSensor::getLongitude() const { return longitude; }

float GPSSensor::getAltitude() const { return altitude; }

uint32_t GPSSensor::getFixType() const { return fix_type; }

bool GPSSensor::getPosLock() const { return pos_lock; }

uint32_t GPSSensor::getSIVCount() const { return SIV_count; }
