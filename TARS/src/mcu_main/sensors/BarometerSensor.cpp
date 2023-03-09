#include "mcu_main/sensors/BarometerSensor.h"

#include "ChRt.h"
#include "MS5611.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/debug.h"

BarometerSensor barometer;

ErrorCode BarometerSensor::init() {
#ifdef ENABLE_BAROMETER
    MS.init();
#endif
    return ErrorCode::NO_ERROR;
}

void BarometerSensor::update() {
#ifdef ENABLE_BAROMETER
    chMtxLock(&mutex);
    MS.read(12);
    pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
    temperature = static_cast<float>(MS.getTemperature() * 0.01);
    altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);
    dataLogger.pushBarometerFifo((BarometerData){temperature, pressure, altitude, chVTGetSystemTime()});
    chMtxUnlock(&mutex);
#endif
}

void BarometerSensor::update(HILSIMPacket hilsim_packet) {
    chMtxLock(&mutex);
    pressure = hilsim_packet.barometer_pressure;
    temperature = hilsim_packet.barometer_temperature;
    altitude = hilsim_packet.barometer_altitude;
    dataLogger.pushBarometerFifo((BarometerData){temperature, pressure, altitude, chVTGetSystemTime()});
    chMtxUnlock(&mutex);
}

float BarometerSensor::getPressure() const { return pressure; }

float BarometerSensor::getTemperature() const { return temperature; }

float BarometerSensor::getAltitude() const { return altitude; }

#ifdef ENABLE_BAROMETER
BarometerSensor::BarometerSensor() : MS{MS5611_CS} { }
#else
BarometerSensor::BarometerSensor() = default;
#endif
