#include "mcu_main/sensors/LowGSensor.h"

#include "common/packet.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/pins.h"

LowGSensor lowG;

void LowGSensor::update() {
     chSysLock();
     chMtxLock(&mutex);

     ax = LSM.readFloatAccelX();
     ay = LSM.readFloatAccelY();
     az = LSM.readFloatAccelZ();
     gx = LSM.readFloatGyroX();
     gy = LSM.readFloatGyroY();
     gz = LSM.readFloatGyroZ();
//     mx = LSM.calcMag(LSM.mx);
//     my = LSM.calcMag(LSM.my);
//     mz = LSM.calcMag(LSM.mz);

     timestamp = chVTGetSystemTime();

     chMtxUnlock(&mutex);
     chSysUnlock();

     dataLogger.pushLowGFifo((LowGData){ax, ay, az, gx, gy, gz, timestamp});
}

Acceleration LowGSensor::getAcceleration() { return Acceleration{ax, ay, az}; }

Gyroscope LowGSensor::getGyroscope() { return Gyroscope{gx, gy, gz}; }
//Magnetometer LowGSensor::getMagnetometer() { return Magnetometer{mx, my, mz}; }

ErrorCode LowGSensor::init() {
    // note, we need to send this our CS pins (defined above)
    if (!LSM.begin()) {
        return ErrorCode::CANNOT_CONECT_LSM9DS1;
    }
    return ErrorCode::NO_ERROR;
}

LowGSensor::LowGSensor() : LSM(SPI_MODE, LSM6DSLTR) { }
