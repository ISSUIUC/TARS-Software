#pragma once

#include "ChRt.h"
#include "MS5611.h"
#include "mcu_main/pins.h"
#include "mcu_main/error.h"
#include "mcu_main/debug.h"

struct BarometerSensor {
   public:
    BarometerSensor();

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
    void refresh();
    float getPressure() const;
    float getTemperature() const;
    float getAltitude() const;

   private:
#ifdef ENABLE_BAROMETER
    MS5611 MS;
#endif
    float pressure = 0.0;
    float temperature = 0.0;
    float altitude = 0.0;
};
