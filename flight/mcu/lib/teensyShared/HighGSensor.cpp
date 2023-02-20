#include "HighGSensor.h"

#include "dataLog.h"
#include "pins.h"

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

void HighGSensor::init() {
    if (!KX.beginSPICore(KX134_CS, 1000000, SPI)) {
        Serial.println("Failed to communicate with KX134. Stalling Program");
        digitalWrite(LED_RED, HIGH);
        while (true)
            ;
    }

    if (!KX.initialize(DEFAULT_SETTINGS)) {
        Serial.println("Could not initialize KX134. Stalling Program");
        digitalWrite(LED_BLUE, HIGH);
        while (true)
            ;
    }

    KX.setRange(3);  // set range to 3 = 64 g range
}
