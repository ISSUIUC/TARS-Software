#pragma once

#include "mcu_main/Rt.h"

#include "mcu_main/sensors/HighGSensor.h"
#include "common/packet.h"
#include "mcu_main/error.h"

#ifdef ENABLE_HILSIM_MODE
#include "mcu_main/hilsim/hilsimpacket.pb.h"
#endif
#ifndef ENABLE_SILSIM_MODE
#include "SparkFunLSM6DS3.h"
#endif

/**
 *
 * @class LowGSensor
 *
 * @brief This class initializes and controls the LowG sensor. One can obtain data using the functions provided in the
 * class.
 *
 * Currently the chip select is given to the default constructor using the
 * LSM9DS1. Using this class one can obtain the current acceleration, gyroscope, and magnetometer data.
 * The range on the low-g sensor is worse than the high-g sensor for acceleration, only -2 to 2gs.
 */
class LowGSensor {
   public:
    MUTEX_DECL(mutex);

    LowGSensor();

    ErrorCode __attribute__((warn_unused_result)) init();

#ifdef ENABLE_HILSIM_MODE
    void update(HILSIMPacket hilsim_packet);
#else
    void update();
#endif
    Acceleration getAcceleration();
    Gyroscope getGyroscope();

   private:
    float ax = 0.0, ay = 0.0, az = 0.0;
    float gx = 0.0, gy = 0.0, gz = 0.0;
    systime_t timestamp = 0;

#ifndef ENABLE_SILSIM_MODE
    LSM6DS3 LSM;
#endif
};
