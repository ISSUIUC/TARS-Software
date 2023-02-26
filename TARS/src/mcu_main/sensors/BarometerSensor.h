#pragma once

#include "ChRt.h"
#include "MS5611.h"
#include "mcu_main/pins.h"
#include "mcu_main/error.h"

struct BarometerSensor {
   public:
    BarometerSensor() = default;

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
    void refresh();
    float getPressure() const;
    float getTemperature() const;
    float getAltitude() const;

   private:
    // We need to store the data of the sensor somewhere. Inside the wrapper class is the best way.
    MS5611 MS{MS5611_CS};
    float pressure = 0.0;
    float temperature = 0.0;
    float altitude = 0.0;
};
