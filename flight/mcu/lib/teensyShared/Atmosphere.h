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

#ifndef _ATMOSPHERE_H_
#define _ATMOSPHERE_H_

class Atmosphere {
   public:
    static double get_temperature(double altitude);
    static double get_pressure(double altitude);
    static double get_density(double altitude);
    static double get_speed_of_sound(double altitude);
    static double get_geometric_to_geopotential(double altitude);
};

#endif
