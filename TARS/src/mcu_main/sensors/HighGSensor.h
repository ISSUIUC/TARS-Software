#pragma once

#include <tuple>

#include "ChRt.h"
#include "SparkFun_Qwiic_KX13X.h"

#include "common/packet.h"

#include "mcu_main/error.h"

struct HighGSensor {
   public:
    HighGSensor() = default;

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
    void update();
    Acceleration getAccel();

   private:
    float ax = 0.0, ay = 0.0, az = 0.0;
    systime_t timestamp = 0;
    QwiicKX134 KX;
};
