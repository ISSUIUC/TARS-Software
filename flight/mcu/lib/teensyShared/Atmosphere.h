/**
 * @file        Atmosphere.h
 * @authors     Kenneth Tochihara
 *              Buffet Lee
 *              Ayberk Yaraneri
 *
 * @brief       Class definitions for the atmosphere model
 *
 * The classes defined here represent atmosphere components. These
 * objects will represent the atmosphere model used for simulation.
 *
 */

#pragma once

class Atmosphere {
   public:
    static double getTemperature(double altitude);
    static double getPressure(double altitude);
    static double getDensity(double altitude);
    static double getSpeedOfSound(double altitude);
    static double getGeometricToGeopotential(double altitude);
};
