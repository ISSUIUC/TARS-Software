/**
 * @file        Atmosphere.cpp
 * @authors     Kenneth Tochihara
 *              Buffet Lee
 *              Ayberk Yaraneri
 *
 * @brief       Member function implementations of atmosphere class
 *
 * The classes defined here represent atmosphere components. These
 * objects will represent the atmosphere model used for simulation.
 *
 */

#include "Atmosphere.h"

#include <cmath>

/**
 * @brief Temperature getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_temperature(double altitude) {
    double temperature;
    double altitude_h =
        get_geometric_to_geopotential(altitude) / 1000;  // geopotential//
    double altitude_z = altitude / 1000;                 // geometric//
    if (altitude_h < 11.0) {
        temperature = 288.15 - (6.5 * altitude_h);
    }

    else if (altitude_h < 20.0) {
        temperature = 216.65;
    }

    else if (altitude_h < 32.0) {
        temperature = 196.65 + altitude_h;
    }

    else if (altitude_h < 47.0) {
        temperature = 139.05 + (2.8 * altitude_h);
    }

    else if (altitude_h < 51.0) {
        temperature = 270.65;
    }

    else if (altitude_h < 71.0) {
        temperature = 413.45 - (2.8 * altitude_h);
    }

    else if (altitude_h < 84.852) {
        temperature = 356.65 - (2.0 * altitude_h);
    }

    else if (altitude_z < 91) {
        temperature = 186.8673;
    }

    else if (altitude_z < 110) {
        temperature =
            263.1905 -
            76.3232 * sqrt(1 - pow((altitude_z - 91) / -19.9429, 2.0));
    }

    else if (altitude_z < 120) {
        temperature = 240 + 12 * (altitude_z - 110);
    }

    else if (altitude_z < 1000) {
        temperature =
            1000 - 640 * exp(-0.01875 * ((altitude_z - 120) * (6356.766 + 120) /
                                         (6356.766 + altitude_z)));
    } else {
        temperature = 0;
    }
    return temperature;
}

/**
 * @brief Pressure getter function based on altitude_h
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_pressure(double altitude) {
    double pressure;
    double altitude_h = get_geometric_to_geopotential(altitude) / 1000;
    double altitude_z = altitude / 1000;

    if (altitude_h < 11) {
        pressure = 101325.0 * pow((288.15 / (288.15 - 6.5 * altitude_h)),
                                  (34.1632 / -6.5));
    }

    else if (altitude_h < 20) {
        pressure = 22632.06 * exp(-34.1632 * (altitude_h - 11) / 216.65);
    }

    else if (altitude_h < 32) {
        pressure =
            5474.889 * pow((216.65 / (216.65 + (altitude_h - 20))), 34.1632);
    }

    else if (altitude_h < 47) {
        pressure = 868.0187 * pow((228.65 / (228.65 + 2.8 * (altitude_h - 32))),
                                  (34.1632 / 2.8));
    }

    else if (altitude_h < 51) {
        pressure = 110.9063 * exp(-34.1632 * (altitude_h - 47) / 270.65);
    }

    else if (altitude_h < 71) {
        pressure = 66.93887 * pow((270.65 / (270.65 - 2.8 * (altitude_h - 51))),
                                  (34.1632 / -2.8));
    }

    else if (altitude_h < 84.852) {
        pressure = 3.956420 * pow((214.65 / (214.65 - 2 * (altitude_h - 71))),
                                  (34.1632 / -2));
    }

    else if (altitude_h < 91) {
        pressure = exp(0.000000 * pow(altitude_h, 4) +
                       2.159582E-06 * pow(altitude_h, 3) +
                       -4.836957E-04 * pow(altitude_h, 2) +
                       -0.1425192 * altitude_h + 13.47530);
    }

    else if (altitude_z < 100) {
        pressure = exp(0.000000 * pow(altitude_z, 4) +
                       3.304895E-05 * pow(altitude_z, 3) +
                       -0.009062730 * pow(altitude_z, 2) +
                       0.6516698 * altitude_z + -11.03037);
    }

    else if (altitude_z < 110) {
        pressure = exp(0.000000 * pow(altitude_z, 4) +
                       6.693926E-05 * pow(altitude_z, 3) +
                       -0.01945388 * pow(altitude_z, 2) +
                       1.719080 * altitude_z + -47.75030);
    }

    else if (altitude_z < 120) {
        pressure = exp(0.000000 * pow(altitude_z, 4) +
                       -6.539316E-05 * pow(altitude_z, 3) +
                       0.02485568 * pow(altitude_z, 2) +
                       -3.223620 * altitude_z + 135.9355);
    }

    else if (altitude_z < 150) {
        pressure = exp(2.283506E-07 * pow(altitude_z, 4) +
                       -1.343221E-04 * pow(altitude_z, 3) +
                       0.02999016 * pow(altitude_z, 2) +
                       -3.055446 * altitude_z + 113.5764);
    }

    else if (altitude_z < 200) {
        pressure = exp(1.209434E-08 * pow(altitude_z, 4) +
                       -9.692458E-06 * pow(altitude_z, 3) +
                       0.003002041 * pow(altitude_z, 2) +
                       -0.4523015 * altitude_z + 19.19151);
    }

    else if (altitude_z < 300) {
        pressure = exp(8.113942E-10 * pow(altitude_z, 4) +
                       -9.822568E-07 * pow(altitude_z, 3) +
                       4.687616E-04 * pow(altitude_z, 2) +
                       -0.1231710 * altitude_z + 3.067409);
    }

    else if (altitude_z < 500) {
        pressure = exp(9.814674E-11 * pow(altitude_z, 4) +
                       -1.654439E-07 * pow(altitude_z, 3) +
                       1.148115E-04 * pow(altitude_z, 2) +
                       -0.05431334 * altitude_z + -2.011365);
    }

    else if (altitude_z < 750) {
        pressure = exp(-7.835161E-11 * pow(altitude_z, 4) +
                       1.964589E-07 * pow(altitude_z, 3) +
                       -1.657213E-04 * pow(altitude_z, 2) +
                       0.04305869 * altitude_z + -14.77132);
    }

    else if (altitude_z < 1000) {
        pressure = exp(2.813255E-11 * pow(altitude_z, 4) +
                       -1.120689E-07 * pow(altitude_z, 3) +
                       1.695568E-04 * pow(altitude_z, 2) +
                       -0.1188941 * altitude_z + 14.56718);
    }

    else {
        pressure = 0;
    }
    // 86k to 1000k formula not sure yet
    return pressure;
}

/**
 * @brief Density getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_density(double altitude) {
    double R = 287.053;
    double pressure = get_pressure(altitude);
    double temperature = get_temperature(altitude);
    double density;
    altitude = altitude / 1000;

    if (altitude < 84.853) {
        density = pressure / (R * temperature);
    }

    else if (altitude < 91) {
        density = exp(
            0.000000 * pow(altitude, 4) + -3.322622E-06 * pow(altitude, 3) +
            9.111460E-04 * pow(altitude, 2) + -0.2609971 * altitude + 5.944694);
    }

    else if (altitude < 100) {
        density = exp(
            0.000000 * pow(altitude, 4) + 2.873405E-05 * pow(altitude, 3) +
            -0.008492037 * pow(altitude, 2) + 0.6541179 * altitude + -23.62010);
    }

    else if (altitude < 110) {
        density = exp(
            -1.240774E-05 * pow(altitude, 4) + 0.005162063 * pow(altitude, 3) +
            -0.8048342 * pow(altitude, 2) + 55.55996 * altitude + -1443.338);
    }

    else if (altitude < 120) {
        density = exp(
            0.00000 * pow(altitude, 4) + -8.854164E-05 * pow(altitude, 3) +
            0.03373254 * pow(altitude, 2) + -4.390837 * altitude + 176.5294);
    }

    else if (altitude < 150) {
        density = exp(
            3.661771E-07 * pow(altitude, 4) + -2.154344E-04 * pow(altitude, 3) +
            0.04809214 * pow(altitude, 2) + -4.884744 * altitude + 172.3597);
    }

    else if (altitude < 200) {
        density = exp(
            1.906032E-08 * pow(altitude, 4) + -1.527799E-05 * pow(altitude, 3) +
            0.004724294 * pow(altitude, 2) + -0.6992340 * altitude + 20.50921);
    }

    else if (altitude < 300) {
        density = exp(1.199282E-09 * pow(altitude, 4) +
                      -1.451051E-06 * pow(altitude, 3) +
                      6.910474E-04 * pow(altitude, 2) + -0.1736220 * altitude +
                      -5.321644);
    }

    else if (altitude < 500) {
        density = exp(1.140564E-10 * pow(altitude, 4) +
                      -2.130756E-07 * pow(altitude, 3) +
                      1.570762E-04 * pow(altitude, 2) + -0.07029296 * altitude +
                      -12.89844);
    }

    else if (altitude < 750) {
        density = exp(8.105631E-12 * pow(altitude, 4) +
                      -2.358417E-09 * pow(altitude, 3) +
                      -2.635110E-06 * pow(altitude, 2) +
                      -0.01562608 * altitude + -20.02246);
    }

    else if (altitude < 1000) {
        density = exp(-3.701195E-12 * pow(altitude, 4) +
                      -8.608611E-09 * pow(altitude, 3) +
                      5.118829E-05 * pow(altitude, 2) + -0.06600998 * altitude +
                      -6.137674);
    }

    else {
        density = 0;
    }

    return density;
}

/**
 * @brief Calculates the speed-of-sound at the current altitude
 *
 * @param altitude Altitude above sea level in meters
 * @return double Speed of sound in m/s
 */
double Atmosphere::get_speed_of_sound(double altitude) {
    constexpr double gamma = 1.4;            // Heat Capacity Ratio of air
    constexpr double gas_constant = 287.05;  // Gas constant of air

    return sqrt(gamma * gas_constant * get_temperature(altitude));
}

double Atmosphere::get_geometric_to_geopotential(double altitude) {
    double r = 6371000;  // r means the radius of Earth//
    double geopotential = (r * altitude) / (r + altitude);

    return geopotential;
}
