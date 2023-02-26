#include "MagnetometerSensor.h"

#include "mcu_main/pins.h"

MagnetometerSensor magnetometer;

void MagnetometerSensor::init() {
    sensor.begin_SPI(LIS3MDL_CS);
    sensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    sensor.setDataRate(LIS3MDL_DATARATE_80_HZ);
    sensor.setRange(LIS3MDL_RANGE_4_GAUSS);
}

void MagnetometerSensor::update() {
    sensor.read();
}

Magnetometer MagnetometerSensor::getMagnetometer() {
    return { sensor.x_gauss, sensor.y_gauss, sensor.z_gauss };
}
