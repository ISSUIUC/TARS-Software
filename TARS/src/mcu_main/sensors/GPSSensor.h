#pragma once

#include "mcu_main/Rt.h"
#include "mcu_main/error.h"
#ifndef ENABLE_SILSIM_MODE
#include "SparkFun_u-blox_GNSS_v3.h"
#endif

/**
 *
 * @class GPSSensor
 *
 * @brief This class initializes and controls the GPS. One can obtain data using the functions provided in the class.
 *
 * This class utilizes a GPS. Currently the chip select is given to the default constructor using the
 * SFE_UBLOX_GNSS. Using this class one can obtain latitude, longitude, and altitude. One can also
 * get the time, fix type, and satellite in view count.
 */
struct GPSSensor {
   public:
    GPSSensor() = default;

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
    void update();
    float getLatitude() const;
    float getLongitude() const;
    float getAltitude() const;
    /**
     * @brief This returns the fix type which refers to the current encoding scheme.
     * @return An integer corresponding to the current fix type.
     *
     * 0 = no fix (GPS cannot get valid location),
     * 3 = 3D (GPS Working normally),
     * 4 = GNSS+Deadreckoning (Has gps lock and using IMU data to increase accuracy)
     */
    uint32_t getFixType() const;
    bool getPosLock() const;

    /**
     * @brief This returns the number of satellites that the GPS is currently connected to.
     * @return sattelite in view count
     */
    uint32_t getSIVCount() const;

   private:
#ifndef ENABLE_SILSIM_MODE
    SFE_UBLOX_GNSS GNSS;
#endif

    systime_t timeStamp{};
    float latitude{};
    float longitude{};
    float altitude{};
    uint32_t fix_type{};
    bool pos_lock{};
    uint32_t SIV_count{};
};
