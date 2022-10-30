#include "ChRt.h"
#include "MS5611.h"
#include "dataLog.h"

#include "BarometerSensor.h"

BarometerSensor barometer{};

void BarometerSensor::init() {
    MS.init();
}

void BarometerSensor::refresh() {
    chMtxLock(&mutex);
    MS.read(12);
    pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
    temperature = static_cast<float>(MS.getTemperature() * 0.01);
    altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);
    dataLogger.pushBarometerFifo((BarometerData) {temperature, pressure, altitude, chVTGetSystemTime()});
    chMtxUnlock(&mutex);
}

float BarometerSensor::getPressure() const {
    return pressure;
}

float BarometerSensor::getTemperature() const {
    return temperature;
}

float BarometerSensor::getAltitude() const {
    return altitude;
}
