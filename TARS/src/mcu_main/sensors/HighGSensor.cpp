#include "mcu_main/sensors/HighGSensor.h"

#include "mcu_main/dataLog.h"
#include "mcu_main/debug.h"
#include "mcu_main/pins.h"

HighGSensor highG;

void HighGSensor::update() {
#ifdef ENABLE_HIGH_G
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
#endif
}

void HighGSensor::update(HILSIMPacket hilsim_packet) {
#ifdef ENABLE_HIGH_G
    chSysLock();
    chMtxLock(&mutex);
    ax = hilsim_packet.imu_high_ax;
    ay = hilsim_packet.imu_high_ay;
    az = hilsim_packet.imu_high_az;

    dataLogger.pushHighGFifo((HighGData){ax, ay, az, chVTGetSystemTime()});

    chMtxUnlock(&mutex);
    chSysUnlock();
#endif
}

Acceleration HighGSensor::getAccel() { return {ax, ay, az}; }

ErrorCode HighGSensor::init() {
#ifdef ENABLE_HIGH_G
    KX.beginSPI(KX134_CS);
    // TODO for some reason it works fine even if beginSPI claims it fails
    //   idk lmao
    //    if (!KX.beginSPI(KX134_CS)) {
    //        return ErrorCode::CANNOT_CONNECT_KX134_CS;
    //    }

    if (!KX.initialize(DEFAULT_SETTINGS)) {
        return ErrorCode::CANNOT_INIT_KX134_CS;
    }

    KX.setRange(3);  // set range to 3 = 64 g range
#endif
    return ErrorCode::NO_ERROR;
}
