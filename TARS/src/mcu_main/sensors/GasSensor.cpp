#include "GasSensor.h"

#include "mcu_main/pins.h"
#include "mcu_main/dataLog.h"
#include "common/packet.h"
#include "mcu_main/emulation.h"

GasSensor gas;

#ifdef ENABLE_SILSIM_MODE
GasSensor::GasSensor() = default;
#else
GasSensor::GasSensor() : bme(BME688_CS) { }
#endif

ErrorCode GasSensor::init() {
#ifndef ENABLE_SILSIM_MODE
    bme.begin();
#endif
    return ErrorCode::NO_ERROR;
}

float GasSensor::readTemperature() {
    return temperature;
}

void GasSensor::refresh() {
#ifdef ENABLE_SILSIM_MODE
    pressure = emulatedMS->get_data();
    temperature = emulatedThermometer->get_data();
    humidity = 0;
    resistance = 0;
    time_stamp = chVTGetSystemTime();
    dataLogger.pushGasFifo((GasData) {temperature, humidity, pressure, resistance, time_stamp});
#else
    int remaining = bme.remainingReadingMillis();
    if (remaining == 0) {
        bme.performReading();
        temperature = bme.temperature;
        humidity = bme.humidity;
        pressure = bme.pressure;
        resistance = bme.gas_resistance;
        time_stamp = chVTGetSystemTime();
        dataLogger.pushGasFifo((GasData){temperature, humidity, pressure, resistance, time_stamp});
    } else if (remaining == -1) {
        bme.beginReading();
    }
#endif
}
