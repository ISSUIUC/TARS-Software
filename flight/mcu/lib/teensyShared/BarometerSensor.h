#ifndef BAROMETERSENSOR_H
#define BAROMETERSENSOR_H

#include "MS5611.h"
#include "pins.h"

struct BarometerSensor {
   public:
    MUTEX_DECL(mutex);

    void init();
    void refresh();
    float getPressure() const;
    float getTemperature() const;
    float getAltitude() const;

   private:
    MS5611 MS{MS5611_CS};
    float pressure = 0.0;
    float temperature = 0.0;
    float altitude = 0.0;
};

#endif
