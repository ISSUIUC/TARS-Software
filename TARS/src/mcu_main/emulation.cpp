#include "mcu_main/emulation.h"

#ifdef ENABLE_SILSIM_MODE
EmulatedMagnetometerSensor* emulatedMagnetometer = nullptr;
GyroscopeSensor* emulatedGyro = nullptr;
EmulatedGPSSensor* emulatedGPS = nullptr;
Barometer* emulatedMS = nullptr;
Thermometer* emulatedThermometer = nullptr;
Accelerometer* emulatedKX = nullptr;
#endif