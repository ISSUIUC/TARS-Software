#ifndef BAROMETERSENSOR_H
#define BAROMETERSENSOR_H

#include <MS5611.h>

struct BarometerSensor {
    MS5611* MS;
    void readReadings();
    float getPressure();
    float getTemperature();
    float getAltitude();
    float pressure;
    float temperature;
    float altitude;
};




#endif
