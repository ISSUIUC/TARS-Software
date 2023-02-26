#pragma once

#include "ChRt.h"
#include "SparkFunLSM6DS3.h"
#include "mcu_main/sensors/HighGSensor.h"
#include "common/packet.h"

class LowGSensor {
   public:
    MUTEX_DECL(mutex);

    LowGSensor();

    void init();
    void update();
    Acceleration getAcceleration();
    Gyroscope getGyroscope();
//    Magnetometer getMagnetometer();

   private:
    float ax = 0.0, ay = 0.0, az = 0.0;
    float gx = 0.0, gy = 0.0, gz = 0.0;
//    float mx = 0.0, my = 0.0, mz = 0.0;
    systime_t timestamp = 0;

    LSM6DS3 LSM;
};
