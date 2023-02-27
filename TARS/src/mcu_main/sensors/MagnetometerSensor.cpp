#include "MagnetometerSensor.h"

#include "mcu_main/pins.h"
#include "mcu_main/dataLog.h"

MagnetometerSensor magnetometer;

ErrorCode MagnetometerSensor::init() {
    if (!sensor.begin_SPI(LIS3MDL_CS)) {
        return ErrorCode::CANNOT_CONNECT_MAGNETOMETER;
    }
    sensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    sensor.setDataRate(LIS3MDL_DATARATE_80_HZ);
    sensor.setRange(LIS3MDL_RANGE_4_GAUSS);
    return ErrorCode::NO_ERROR;
}

void MagnetometerSensor::update() {
    Serial.println("reading...");
    sensor.read();
    Serial.println("read");

    time_stamp = chVTGetSystemTime();
    dataLogger.pushMagnetometerFifo((MagnetometerData) { {sensor.x_gauss, sensor.y_gauss, sensor.z_gauss}, time_stamp });
}

Magnetometer MagnetometerSensor::getMagnetometer() {
    return { sensor.x_gauss, sensor.y_gauss, sensor.z_gauss };
}
