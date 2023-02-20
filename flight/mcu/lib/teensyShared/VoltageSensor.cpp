#include "VoltageSensor.h"

#include "ChRt.h"
#include "dataLog.h"

VoltageSensor voltage{Serial1};

VoltageData VoltageSensor::read() {
    chMtxLock(&mutex);

    v_battery = read_voltage('B');
    v_servo1 = read_voltage('1');
    v_servo2 = read_voltage('2');
    v_3_3 = read_voltage('3');
    v_5 = read_voltage('5');
    v_9 = read_voltage('9');
    timestamp = chVTGetSystemTime();

    auto data = (VoltageData){v_battery, v_servo1, v_servo2, v_3_3, v_5, v_9, timestamp};

    dataLogger.pushVoltageFifo(data);

    chMtxUnlock(&mutex);

    return data;
}
