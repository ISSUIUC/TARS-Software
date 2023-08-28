#pragma once

#include "mcu_main/Rt.h"
#include "mcu_main/error.h"

#ifndef ENABLE_SILSIM_MODE
#include <Adafruit_BME680.h>
#endif

class GasSensor {
public:
    GasSensor();

    ErrorCode __attribute__((warn_unused_result)) init();
    void refresh();

    float readTemperature();

   private:
#ifndef ENABLE_SILSIM_MODE
    Adafruit_BME680 bme;
#endif

    float temperature = 0.0;
    float humidity = 0.0;
    uint32_t pressure = 0;
    uint32_t resistance = 0;
    systime_t time_stamp = 0;
};