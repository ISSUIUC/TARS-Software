#include "VoltageSensor.h"

#include "dataLog.h"

#include "ChRt.h"

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

    dataLogger.pushVoltageFifo((VoltageData) {v_battery, v_servo1, v_servo2, v_3_3, v_5, v_9, timestamp});

    chMtxUnlock(&mutex);
}
