#pragma once

#include <tuple>

#include "mcu_main/Rt.h"

#include "common/packet.h"
#include "mcu_main/error.h"
#include "mcu_main/hilsim/hilsimpacket.pb.h"

#ifndef ENABLE_SILSIM_MODE
#include "SparkFun_Qwiic_KX13X.h"
#endif

/**
 *
 * @class HighGSensor
 *
 * @brief This class initializes and controls the HighG sensor. One can obtain data using the functions provided in the
 * class.
 *
 * Currently the chip select is given to the default constructor using the
 * QuiicKX132. Using this class one can obtain the current acceleration.
 * The range on the high-g sensor is better than the low-g sensor, with a range higher than could reasonably be
 * obtained.
 */

struct HighGSensor {
   public:
    HighGSensor() = default;

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
    void update();
    void update(HILSIMPacket hilsim_packet);
    Acceleration getAccel();

   private:
    float ax = 0.0, ay = 0.0, az = 0.0;
    systime_t timestamp = 0;
#ifndef ENABLE_SILSIM_MODE
    QwiicKX134 KX;
#endif
};
