#include "mcu_main/sensors/HighGSensor.h"

#include "mcu_main/dataLog.h"
#include "mcu_main/pins.h"

HighGSensor highG;

void HighGSensor::update() {
    chSysLock();
    chMtxLock(&mutex);
    auto data = KX.getAccelData();
    ax = data.xData;
    ay = data.yData;
    az = data.zData;
    timestamp = chVTGetSystemTime();
    dataLogger.pushHighGFifo((HighGData){ax, ay, az, timestamp});

    chMtxUnlock(&mutex);
    chSysUnlock();
}

Acceleration HighGSensor::getAccel() { return {ax, ay, az}; }

ErrorCode HighGSensor::init() {
    if (!KX.beginSPI(KX134_CS)) {
        return ErrorCode::CANNOT_CONNECT_KX134_CS;
    }

    if (!KX.initialize(DEFAULT_SETTINGS)) {
        return ErrorCode::CANNOT_INIT_KX134_CS;
    }

    KX.setRange(3);  // set range to 3 = 64 g range
    return ErrorCode::NO_ERROR;
}
