#ifndef HIGHGSENSOR_H
#define HIGHGSENSOR_H

#include <KX134-1211.h>

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
    KX134* KX;
};

#endif
