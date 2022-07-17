#ifndef HIGHGSENSOR_H
#define HIGHGSENSOR_H

#include "SparkFun_Qwiic_KX13X.h"

struct GForce {
    float x_gforce;
    float y_gforce;
    float z_gforce;
};
class HighGSensor {
   public:
    void readReadings();
    GForce getGForce();

   private:
    float ax, ay, az;
    QwiicKX132* KX;
};

#endif
