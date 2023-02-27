#include "GasSensor.h"

#include "mcu_main/pins.h"
#include "mcu_main/dataLog.h"
#include "common/packet.h"

GasSensor gas;

GasSensor::GasSensor() : bme(BME688_CS) { }

ErrorCode GasSensor::init() {
    bme.begin();
    return ErrorCode::NO_ERROR;
}

float GasSensor::readTemperature() {
    return temperature;
}

void GasSensor::refresh() {
    temperature = bme.readTemperature();
    time_stamp = chVTGetSystemTime();
    dataLogger.pushGasFifo((GasData) { temperature, time_stamp });
}
