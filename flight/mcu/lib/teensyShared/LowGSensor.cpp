#include <SparkFunLSM9DS1.h>
#include <ChRt.h>
#include "LowGSensor.h"

void LowGSensor::readReadings() {
    chSysLock();
    LSM->readAccel();
    LSM->readGyro();
    LSM->readMag();
    chSysUnlock();
}