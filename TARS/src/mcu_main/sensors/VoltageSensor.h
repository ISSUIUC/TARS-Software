#pragma once

#include "Arduino.h"
#include "ChRt.h"
#include "common/packet.h"

class VoltageSensor {
   public:
    MUTEX_DECL(mutex);

    explicit VoltageSensor() {
        pinMode(16, INPUT);
    };

    VoltageData read();

   private:
    float v_battery = 0.0;
    systime_t timestamp = 0;
};
