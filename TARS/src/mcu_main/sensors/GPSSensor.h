#pragma once

#include "ChRt.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "mcu_main/error.h"

struct GPSSensor {
   public:
    GPSSensor() = default;

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
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
    float latitude{};
    float longitude{};
    float altitude{};
    uint32_t fix_type{};
    bool pos_lock{};
    uint32_t SIV_count{};
};
