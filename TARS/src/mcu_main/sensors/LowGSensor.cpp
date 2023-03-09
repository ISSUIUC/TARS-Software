#include "mcu_main/sensors/LowGSensor.h"

#include "common/packet.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/pins.h"
#include "mcu_main/debug.h"
#include "mcu_main/hilsim/HILSIMPacket.h"

LowGSensor lowG;

void LowGSensor::update() {
#ifdef ENABLE_LOW_G
    chSysLock();
    chMtxLock(&mutex);

    ax = LSM.readFloatAccelX();
    ay = LSM.readFloatAccelY();
    az = LSM.readFloatAccelZ();
    gx = LSM.readFloatGyroX();
    gy = LSM.readFloatGyroY();
    gz = LSM.readFloatGyroZ();

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
//Magnetometer LowGSensor::getMagnetometer() { return Magnetometer{mx, my, mz}; }

ErrorCode LowGSensor::init() {
#ifdef ENABLE_LOW_G
    // note, we need to send this our CS pins (defined above)
    if (!LSM.begin()) {
        return ErrorCode::CANNOT_CONNECT_LSM9DS1;
    }
#endif
    return ErrorCode::NO_ERROR;
}

#ifdef ENABLE_LOW_G
LowGSensor::LowGSensor() : LSM(SPI_MODE, LSM6DSLTR) { }
#else
LowGSensor::LowGSensor() = default;
#endif
