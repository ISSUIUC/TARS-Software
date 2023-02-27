#pragma once

#include <Adafruit_BME680.h>

#include "mcu_main/error.h"

class GasSensor {
public:
    GasSensor();

    ErrorCode __attribute__((warn_unused_result)) init();
    void refresh();

    float readTemperature();

private:
    Adafruit_BME680 bme;
};