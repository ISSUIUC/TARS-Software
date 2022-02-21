#include "LowGSensor.h"

#include <ChRt.h>
#include <SparkFunLSM9DS1.h>

void LowGSensor::readReadings() {
    chSysLock();
    LSM->readAccel();
    LSM->readGyro();
    LSM->readMag();
    chSysUnlock();
}

Acceleration LowGSensor::getAcceleration() {
    float ax = LSM->calcAccel(LSM->ax);
    float ay = LSM->calcAccel(LSM->ay);
    float az = LSM->calcAccel(LSM->az);

    return Acceleration{ax, ay, az};
}

Gyroscope LowGSensor::getGyroscope() {
    float gx = LSM->calcGyro(LSM->gx);
    float gy = LSM->calcGyro(LSM->gy);
    float gz = LSM->calcGyro(LSM->gz);

    return Gyroscope{gx, gy, gz};
}
Magnetometer LowGSensor::getMagnetometer() {
    float mx = LSM->calcMag(LSM->mx);
    float my = LSM->calcMag(LSM->my);
    float mz = LSM->calcMag(LSM->mz);

    return Magnetometer{mx, my, mz};
}
