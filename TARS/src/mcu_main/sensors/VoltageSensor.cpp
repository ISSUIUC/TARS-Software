#include "VoltageSensor.h"

#include "mcu_main/Rt.h"
#include "mcu_main/dataLog.h"

VoltageSensor voltage;

VoltageData VoltageSensor::read() {
    chMtxLock(&mutex);

#ifdef ENABLE_SILSIM_MODE
    v_battery = 9.0;
#elif
    v_battery = analogRead(16) / 1024.f * 3.3f * 3.f;
#endif
    timestamp = chVTGetSystemTime();

    auto data = (VoltageData){v_battery, timestamp};

    dataLogger.pushVoltageFifo(data);

    chMtxUnlock(&mutex);

    return data;
}
