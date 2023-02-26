#include "GasSensor.h"

#include "mcu_main/pins.h"

GasSensor gasSensor;

GasSensor::GasSensor() : bme(BME688_CS) { }

void GasSensor::init() {
    bme.begin();
}

float GasSensor::readTemperature() {
    return bme.readTemperature();
}

void GasSensor::refresh() {

}
