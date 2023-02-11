#pragma once

#include "ChRt.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

struct GPSSensor {
   public:
    GPSSensor() = default;

    MUTEX_DECL(mutex);

    void init();
    void update();
    float getLatitude() const;
    float getLongitude() const;
    float getAltitude() const;
    uint32_t getFixType() const;
    bool getPosLock() const;
    uint32_t getSIVCount() const;

   private:
    SFE_UBLOX_GNSS GNSS;

    systime_t timeStamp{};
    float latitude = 0.0;
    float longitude = 0.0;
    float altitude = 0.0;
    uint32_t fix_type{};
    bool pos_lock{};
    uint32_t SIV_count{};
};
