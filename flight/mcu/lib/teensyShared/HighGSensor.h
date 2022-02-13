#ifndef HIGHGSENSOR_H
#define HIGHGSENSOR_H

#include <KX134-1211.h>

struct HighGSensor {
    KX134* KX;
    void readReadings();
    GForce getGForce();
};

struct GForce {
    float x_gforce;
    float y_gforce;
    float z_gforce;
};

#endif