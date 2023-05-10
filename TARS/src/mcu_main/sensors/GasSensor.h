#pragma once

#include <ChRt.h>
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

    float temperature = 0.0;
    float humidity = 0.0;
    uint32_t pressure = 0;
    uint32_t resistance = 0;
    systime_t time_stamp = 0;
};