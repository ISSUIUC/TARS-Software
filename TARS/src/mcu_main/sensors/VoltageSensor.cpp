#include "VoltageSensor.h"

#include "ChRt.h"
#include "mcu_main/dataLog.h"

VoltageSensor voltage;

VoltageData VoltageSensor::read() {
    chMtxLock(&mutex);

    v_battery = analogRead(16) / 1024.f * 3.3f * 3.f;
    timestamp = chVTGetSystemTime();

    auto data = (VoltageData){v_battery, timestamp};

    dataLogger.pushVoltageFifo(data);

    chMtxUnlock(&mutex);

    return data;
}
