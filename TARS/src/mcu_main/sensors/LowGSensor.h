#pragma once

#include "ChRt.h"
#include "SparkFunLSM9DS1.h"
#include "mcu_main/sensors/HighGSensor.h"
#include "mcu_main/error.h"

struct Gyroscope {
    float gx;
    float gy;
    float gz;
};

struct Magnetometer {
    float mx;
    float my;
    float mz;
};

class LowGSensor {
   public:
    MUTEX_DECL(mutex);

    LowGSensor() = default;

    ErrorCode __attribute__((warn_unused_result)) init();
    void update();
    Acceleration getAcceleration();
    Gyroscope getGyroscope();
    Magnetometer getMagnetometer();

   private:
    float ax = 0.0, ay = 0.0, az = 0.0;
    float gx = 0.0, gy = 0.0, gz = 0.0;
    float mx = 0.0, my = 0.0, mz = 0.0;
    systime_t timestamp = 0;

    LSM9DS1 LSM;
};
