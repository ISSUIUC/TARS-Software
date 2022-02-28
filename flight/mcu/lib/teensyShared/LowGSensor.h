#ifndef LOWGSENSOR_H
#define LOWGSENSOR_H

#include <SparkFunLSM9DS1.h>

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

class LowGSensor {
   public:
    void readReadings();
    Acceleration getAcceleration();
    Gyroscope getGyroscope();
    Magnetometer getMagnetometer();

   private:
    LSM9DS1* LSM;
};

#endif
