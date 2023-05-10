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
    int remaining = bme.remainingReadingMillis();
    if (remaining == 0) {
        bme.performReading();
        temperature = bme.temperature;
        humidity = bme.humidity;
        pressure = bme.pressure;
        resistance = bme.gas_resistance;
        time_stamp = chVTGetSystemTime();
        dataLogger.pushGasFifo((GasData) {temperature, humidity, pressure, resistance, time_stamp});
    } else if (remaining == -1) {
        bme.beginReading();
    }
}
