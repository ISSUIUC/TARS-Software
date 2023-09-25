#pragma once

#include "common/packet.h"
#include "mcu_main/error.h"
#ifdef ENABLE_HILSIM_MODE
#include "mcu_main/hilsim/hilsimpacket.pb.h"
#endif
#ifndef ENABLE_SILSIM_MODE
#include <Adafruit_LIS3MDL.h>
#endif

class MagnetometerSensor {
public:
#ifdef ENABLE_HILSIM_MODE
    void update(HILSIMPacket hilsim_packet);
#else
    void update();
#endif
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