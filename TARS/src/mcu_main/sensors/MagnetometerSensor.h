#pragma once

#include "common/packet.h"
#include "mcu_main/error.h"
#include "mcu_main/hilsim/HILSIMPacket.h"
#ifndef ENABLE_SILSIM_MODE
#include <Adafruit_LIS3MDL.h>
#endif

class MagnetometerSensor {
   public:
    void update();
    void update(HILSIMPacket hilsim_packet);
    ErrorCode __attribute__((warn_unused_result)) init();

    Magnetometer getMagnetometer();

   private:
#ifndef ENABLE_SILSIM_MODE
    Adafruit_LIS3MDL sensor;
#endif
    float mx;
    float my;
    float mz;

    systime_t time_stamp = 0;
};