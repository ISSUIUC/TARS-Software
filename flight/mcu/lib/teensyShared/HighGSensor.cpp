#include "HighGSensor.h"

#include <ChRt.h>
#include <KX134-1211.h>

void HighGSensor::readReadings() {
    chSysLock();
    KX->update_data();
    chSysUnlock();
}

GForce HighGSensor::getGForce() {
    float x_gforce = KX->get_x_gforce();
    float y_gforce = KX->get_y_gforce();
    float z_gforce = KX->get_z_gforce();

    return GForce{x_gforce, y_gforce, z_gforce};
}
