#include "HighGSensor.h"

#include <ChRt.h>

void HighGSensor::readReadings() {
    auto data = KX->getAccelData();
    ax = data.xData;
    ay = data.yData;
    az = data.zData;
}

GForce HighGSensor::getGForce() {
    return {ax, ay, az};
}
