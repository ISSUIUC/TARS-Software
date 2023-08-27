#pragma once

#include "Arduino.h"
#include "ChRt.h"
#include "common/packet.h"

/**
 *
 * @class VoltageSensor
 *
 * @brief This class initializes and controls the voltage sensor. One can obtain data using the functions provided in
 * the class.
 *
 * The constructor is default. One can get the current voltage using the
 * read() function which directly performs an analogRead of a pin routed to the
 * Power Management Board. This is an update from 2022 when we had an ATMega on the PMB
 * send the voltage data through serial
 */
class VoltageSensor {
   public:
    MUTEX_DECL(mutex);

    explicit VoltageSensor() { pinMode(16, INPUT); };

    VoltageData read();

   private:
    float v_battery = 0.0;
    systime_t timestamp = 0;
};
