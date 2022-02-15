#ifndef LOWGSENSOR_H
#define LOWGSENSOR_H

#include <SparkFunLSM9DS1.h>

struct LowGSensor {
    LSM9DS1* LSM;
    void readReadings();
    Acceleration getAcceleration();
    Gyroscope getGyroscope();
    Magnetometer getMagnetometer();
};

struct Acceleration {
    float ax;
    float ay;
    float az;
};

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



#endif