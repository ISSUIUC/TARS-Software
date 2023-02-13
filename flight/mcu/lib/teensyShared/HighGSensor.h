#pragma once

#include <tuple>
#include "ChRt.h"

#include "SparkFun_Qwiic_KX13X.h"

struct Acceleration {
    float ax, ay, az;
};

struct HighGSensor {
public:
    HighGSensor() = default;

    MUTEX_DECL(mutex);

    void init();
    void update();
    Acceleration getAccel();

private:
    float ax = 0.0, ay = 0.0, az = 0.0;
    systime_t timestamp = 0;
    QwiicKX132 KX{};
};
