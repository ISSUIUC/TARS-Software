#include "MagnetometerSensor.h"

#include "mcu_main/Rt.h"

#include "mcu_main/pins.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/emulation.h"

MagnetometerSensor magnetometer;

ErrorCode MagnetometerSensor::init() {
#ifndef ENABLE_SILSIM_MODE
    if (!sensor.begin_SPI(LIS3MDL_CS)) {
        return ErrorCode::CANNOT_CONNECT_MAGNETOMETER;
    }
    sensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    sensor.setDataRate(LIS3MDL_DATARATE_80_HZ);
    sensor.setRange(LIS3MDL_RANGE_4_GAUSS);
#endif
    return ErrorCode::NO_ERROR;
}

void MagnetometerSensor::update() {
#ifdef ENABLE_MAGNETOMETER
#ifdef ENABLE_SILSIM_MODE
    mx = emulatedMagnetometer->get_data().x();
    my = emulatedMagnetometer->get_data().y();
    mz = emulatedMagnetometer->get_data().z();
#else
    sensor.read();

    mx = sensor.x_gauss;
    my = sensor.y_gauss;
    mz = sensor.z_gauss;
#endif
    time_stamp = chVTGetSystemTime();
    dataLogger.pushMagnetometerFifo((MagnetometerData){{mx, my, mz}, time_stamp});
#endif
}

void MagnetometerSensor::update(HILSIMPacket hilsim_packet) {
#ifdef ENABLE_MAGNETOMETER
    time_stamp = chVTGetSystemTime();
    mx = hilsim_packet.mag_x;
    my = hilsim_packet.mag_y;
    mz = hilsim_packet.mag_z;
    dataLogger.pushMagnetometerFifo((MagnetometerData){{mx, my, mz}, time_stamp});
#endif
}

Magnetometer MagnetometerSensor::getMagnetometer() { return {mx, my, mz}; }
