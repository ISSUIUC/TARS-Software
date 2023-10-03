#pragma once

#include "mcu_main/Rt.h"
#include "mcu_main/pins.h"
#include "mcu_main/error.h"
#include "mcu_main/debug.h"
#include "mcu_main/hilsim/hilsimpacket.pb.h"
#ifndef ENABLE_SILSIM_MODE
#include "MS5611.h"
#endif

/**
 *
 * @class BarometerSensor
 *
 * @brief This class initializes and controls the barometer. One can obtain data using the functions provided in the
 * class.
 *
 * This class utilizes a barometer. Currently the chip select is given to the default constructor using the
 * MS5611. Using this class one can obtain temperature, pressure, and altitude.
 */
struct BarometerSensor {
   public:
    BarometerSensor();

    MUTEX_DECL(mutex);

    ErrorCode __attribute__((warn_unused_result)) init();
    void update();
    void update(HILSIMPacket hilsim_packet);

    float getPressure() const;
    float getTemperature() const;
    float getAltitude() const;

   private:
#ifdef ENABLE_BAROMETER
#ifndef ENABLE_SILSIM_MODE
    MS5611 MS;
#endif
#endif
    float pressure = 0.0;
    float temperature = 0.0;
    float altitude = 0.0;
};
