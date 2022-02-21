#include <ChRt.h>
#include <MS5611.h>

#include "BarometerSensor.h"

void BarometerSensor::readReadings() { MS->read(12); }

float BarometerSensor::getPressure() {
    pressure = MS->getPressure() * 0.01;
    return pressure;
}

float BarometerSensor::getTemperature() {
    temperature = MS->getTemperature() * 0.01;
    return temperature;
}

float BarometerSensor::getAltitude() {
    altitude = -log(pressure * 0.000987) * (temperature + 273.15) * 29.254;
    return altitude;
}
