#ifndef GPSSENSOR_H
#define GPSSENSOR_H

#include <ChRt.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

struct GPSSensor {
    SFE_UBLOX_GNSS* GNSS;
    void readReadings();
    systime_t timeStamp_GPS;
    float latitude;
    float longtitude;
    float altitude;
    uint32_t fix_type;
    bool pos_lock;
    uint32_t SIV_count;
    float getLatitude();
    float getLongtitude();
    float getAltitude();
    uint32_t getFixType();
    bool getPosLock();
    uint32_t getSIVCount();
};

#endif
