#include "mcu_main/sensors/BarometerSensor.h"

#include "mcu_main/Rt.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/debug.h"

#ifdef ENABLE_SILSIM_MODE
#include "mcu_main/emulation.h"
#endif

BarometerSensor barometer;

ErrorCode BarometerSensor::init() {
#ifndef ENABLE_SILSIM_MODE
#ifdef ENABLE_BAROMETER
    MS.init();
#endif
#endif
    return ErrorCode::NO_ERROR;
}

void BarometerSensor::update() {
    chMtxLock(&mutex);
#ifdef ENABLE_SILSIM_MODE
    pressure = emulatedMS->get_data();
    // std::cout << pressure << std::endl;
    temperature = emulatedThermometer->get_data();
#else
#ifdef ENABLE_BAROMETER
    MS.read(12);
    pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
    temperature = static_cast<float>(MS.getTemperature() * 0.01);
#endif
#endif
    altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);
    dataLogger.pushBarometerFifo((BarometerData){temperature, pressure, altitude, chVTGetSystemTime()});
    chMtxUnlock(&mutex);
}

void BarometerSensor::update(HILSIMPacket hilsim_packet) {
#ifdef ENABLE_BAROMETER
    chMtxLock(&mutex);
    pressure = hilsim_packet.barometer_pressure;
    temperature = hilsim_packet.barometer_temperature;
    altitude = hilsim_packet.barometer_altitude;
    dataLogger.pushBarometerFifo((BarometerData){temperature, pressure, altitude, chVTGetSystemTime()});
    chMtxUnlock(&mutex);
#endif
}

float BarometerSensor::getPressure() const { return pressure; }

float BarometerSensor::getTemperature() const { return temperature; }

float BarometerSensor::getAltitude() const { return altitude; }

#if defined(ENABLE_BAROMETER) && !defined(ENABLE_SILSIM_MODE)
BarometerSensor::BarometerSensor() : MS{MS5611_CS} {}
#else
BarometerSensor::BarometerSensor() = default;
#endif
