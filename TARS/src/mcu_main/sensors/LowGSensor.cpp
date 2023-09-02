#include "mcu_main/sensors/LowGSensor.h"

#include "common/packet.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/pins.h"
#include "mcu_main/debug.h"
#include "mcu_main/hilsim/HILSIMPacket.pb.h"

#ifdef ENABLE_SILSIM_MODE
#include "mcu_main/emulation.h"
#endif

LowGSensor lowG;

void LowGSensor::update() {
#ifdef ENABLE_LOW_G
    chSysLock();
    chMtxLock(&mutex);

#ifdef ENABLE_SILSIM_MODE
    ax = emulatedKX->get_data().x();
    ay = emulatedKX->get_data().y();
    az = emulatedKX->get_data().z();
    gx = emulatedGyro->get_data().x();
    gy = emulatedGyro->get_data().y();
    gz = emulatedGyro->get_data().z();
#else
    ax = LSM.readFloatAccelX();
    ay = LSM.readFloatAccelY();
    az = LSM.readFloatAccelZ();
    gx = LSM.readFloatGyroX();
    gy = LSM.readFloatGyroY();
    gz = LSM.readFloatGyroZ();
#endif

    timestamp = chVTGetSystemTime();

    chMtxUnlock(&mutex);
    chSysUnlock();

    dataLogger.pushLowGFifo((LowGData){ax, ay, az, gx, gy, gz, timestamp});
#endif
}

void LowGSensor::update(HILSIMPacket hilsim_packet) {
#ifdef ENABLE_LOW_G
    chSysLock();
    chMtxLock(&mutex);
    ax = hilsim_packet.imu_low_ax;
    ay = hilsim_packet.imu_low_ay;
    az = hilsim_packet.imu_low_az;
    gx = hilsim_packet.imu_low_gx;
    gy = hilsim_packet.imu_low_gy;
    gz = hilsim_packet.imu_low_gz;

    dataLogger.pushLowGFifo((LowGData){ax, ay, az, gx, gy, gz, chVTGetSystemTime()});

    chMtxUnlock(&mutex);
    chSysUnlock();
#endif
}

Acceleration LowGSensor::getAcceleration() { return Acceleration{ax, ay, az}; }

Gyroscope LowGSensor::getGyroscope() { return Gyroscope{gx, gy, gz}; }
// Magnetometer LowGSensor::getMagnetometer() { return Magnetometer{mx, my, mz}; }

ErrorCode LowGSensor::init() {
#if defined(ENABLE_LOW_G) && !defined(ENABLE_SILSIM_MODE)
    // note, we need to send this our CS pins (defined above)
    if (!LSM.begin()) {
        return ErrorCode::CANNOT_CONNECT_LSM9DS1;
    }
#endif
    return ErrorCode::NO_ERROR;
}

#if defined(ENABLE_LOW_G) && !defined(ENABLE_SILSIM_MODE)
LowGSensor::LowGSensor() : LSM(SPI_MODE, LSM6DSLTR) { }
#else
LowGSensor::LowGSensor() = default;
#endif
