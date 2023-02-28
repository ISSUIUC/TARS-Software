#pragma once

#include <Adafruit_LIS3MDL.h>
#include "common/packet.h"
#include "mcu_main/error.h"


class MagnetometerSensor {
public:
    void update();
    ErrorCode init();

    Magnetometer getMagnetometer();

private:
    Adafruit_LIS3MDL sensor;

    systime_t time_stamp = 0;
};