#pragma once

#include "mcu_main/Rt.h"

#include "mcu_main/sensors/HighGSensor.h"
#include "common/packet.h"
#include "mcu_main/error.h"
#include "mcu_main/hilsim/HILSIMPacket.h"
#ifndef ENABLE_SILSIM_MODE
#include "SparkFunLSM6DS3.h"
#endif

/**
*
* @class LowGSensor
*
* @brief This class initializes and controls the LowG sensor. One can obtain data using the functions provided in the class.
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
    void update();
    void update(HILSIMPacket hilsim_packet);
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
