#include "VoltageSensor.h"

#include "ChRt.h"
#include "mcu_main/dataLog.h"

VoltageSensor voltage{Serial1};

VoltageData VoltageSensor::read() {
    chMtxLock(&mutex);

    v_battery = read_voltage('B');
    timestamp = chVTGetSystemTime();

    auto data = (VoltageData){v_battery, timestamp};

    dataLogger.pushVoltageFifo(data);

    chMtxUnlock(&mutex);

    return data;
}
