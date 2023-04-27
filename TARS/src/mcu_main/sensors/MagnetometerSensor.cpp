#include "MagnetometerSensor.h"

#include "mcu_main/dataLog.h"
#include "mcu_main/debug.h"
#include "mcu_main/pins.h"

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
    // Serial.println("hi");
    #ifdef ENABLE_MAGNETOMETER
    sensor.read();

    time_stamp = chVTGetSystemTime();
    mx = sensor.x_gauss;
    my = sensor.y_gauss;
    mz = sensor.z_gauss;
    // Serial.println("Updated magnetometer");
    dataLogger.pushMagnetometerFifo((MagnetometerData) { {mx, my, mz}, time_stamp });
    #endif
}

void MagnetometerSensor::update(HILSIMPacket hilsim_packet) {
#ifdef ENABLE_MAGNETOMETER
    time_stamp = chVTGetSystemTime();
    mx = hilsim_packet.mag_x;
    my = hilsim_packet.mag_y;
    mz = hilsim_packet.mag_z;
    dataLogger.pushMagnetometerFifo((MagnetometerData) { {mx, my, mz}, time_stamp });

#endif 
}

Magnetometer MagnetometerSensor::getMagnetometer() {
    return { sensor.x_gauss, sensor.y_gauss, sensor.z_gauss };
}
