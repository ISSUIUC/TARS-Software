#ifndef BAROMETERSENSOR_H
#define BAROMETERSENSOR_H

#include <MS5611.h>

class BarometerSensor {
   public:
    void readReadings();
    float getPressure();
    float getTemperature();
    float getAltitude();

   private:
    MS5611* MS;
    float pressure;
    float temperature;
    float altitude;
};

#endif
