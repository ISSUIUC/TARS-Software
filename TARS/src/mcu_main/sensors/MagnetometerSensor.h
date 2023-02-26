#pragma once

#include <Adafruit_LIS3MDL.h>
#include "common/packet.h"


class MagnetometerSensor {
public:
    void update();
    void init();

    Magnetometer getMagnetometer();

private:
    Adafruit_LIS3MDL sensor;
};