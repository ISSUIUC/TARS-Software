#pragma once

#include <Adafruit_LIS3MDL.h>

#include "common/packet.h"
#include "mcu_main/error.h"
#include "mcu_main/hilsim/HILSIMPacket.h"

class MagnetometerSensor {
   public:
    void update();
    void update(HILSIMPacket hilsim_packet);
    ErrorCode __attribute__((warn_unused_result)) init();

    Magnetometer getMagnetometer();

   private:
    Adafruit_LIS3MDL sensor;
    float mx;
    float my;
    float mz;

    systime_t time_stamp = 0;
};