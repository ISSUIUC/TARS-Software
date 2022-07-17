/**
 * @file rk4.cpp
 *
 *Contains the code to dynamically predict the apogee using data from the kalman
 *filter
 */
#include "rk4.h"

#include <Arduino.h>

#include <array>
#include <cmath>

// TODO: make a typedef for array<float, 2>
// using std::array;

rk4::rk4() {
    Atmosphere atmo;
    atmo_ = atmo;
}

/**
 * @brief A function that returns the Coefficient of drag at a given altitude
 * and velocity (based on mach number)
 *
 * @param alt the current altitude from the kalman filter
 * @param vel the current velocity from the kalman filter
 * @return float the Coefficient of drag at the present estimated state
 */
float rk4::cd(float alt, float vel) {
    float mach = vel / (atmo_.get_speed_of_sound(alt));

    double cd = 0;

    double mach_power = 1;
    for (int i = 0; i < 151; i++) {
        cd += poly[150 - i] * mach_power;
        mach_power *= mach;
    }
    return float(cd);
}

/**
 * @brief A function that calculates the acceleration of the rocket at a given
 * altitude and velocity given by the rk4 simulation
 *
 * @param u an array containing the altitude and vertical velocity from the rk4
 * apogee simulation
 * @param rho the air density returned from the atmosphere class
 * @return array<float, 2> the velocity and acceleration (fixed frame) due to
 * aerodynamic forces at the current step of the rk4
 */
array<float, 2> rk4::accel(array<float, 2> u, float rho) {
    float r1 = u[0];
    float v1 = u[1];

    // Approximation - use the area of a circle for reference area
    float Sref_a = .007854;
    float Cd_total = cd(r1, v1);

    float F_a = -((rho * (v1 * v1) * Sref_a * Cd_total) / 2);
    float accel_a = F_a / 21;  // Acceleration due to Aerodynamic Forces
    float accel_f = accel_a - 9.81;

    array<float, 2> f1{v1, accel_f};

    return f1;
}

/**
 * @brief A function that propagates the rk4 apogee simulation by one time step
 *
 * @param state the rk4 altitude and velocity
 * @param dt the time step size
 * @param rho the density of the air at a given rk4 step altitude
 * @return array<float, 2> the next rk4 state
 */
array<float, 2> rk4::rk4_step(array<float, 2> state, float dt, float rho) {
    // rk4 iteration
    array<float, 2> y1 = accel(state, rho);

    array<float, 2> u1 = {(float)(state[0] + .5 * dt * y1[0]),
                          (float)(state[1] + .5 * dt * y1[1])};
    array<float, 2> u2 = {(float)(state[0] + .5 * dt * y2[0]),
                          (float)(state[1] + .5 * dt * y2[1])};
    array<float, 2> u3 = {(float)(state[0] + dt * y3[0]),
                          (float)(state[1] + dt * y3[1])};

    array<float, 2> y2 = accel(u1, rho);
    array<float, 2> y3 = accel(u2, rho);
    array<float, 2> y4 = accel(u3, rho);

    array<float, 2> temp = {
        state[0] + (dt * (y1[0] + 2 * y2[0] + 2 * y3[0] + y4[0])) / 6,
        state[1] + (dt * (y1[1] + 2 * y2[1] + 2 * y3[1] + y4[1])) / 6};

    // rk4_kp1 = state + dt*(y1 + 2*y2 + 2*y3 + y4)/6;
    return temp;
}
/**
 * @brief A function that returns the simulated apogee of the rocket from state
 * estimates
 *
 * @param state altitude and velocity from kalman filter
 * @param dt the time step size
 * @return array<float, 2> the predicted apogee and velocity at that altitude
 * (should be close to zero)
 */
array<float, 2> rk4::sim_apogee(array<float, 2> state, float dt) {
    for (int iters = 0; iters < 120 && state[1] > 0; iters++) {
        // grabbing the current states (I commented these out because they were
        // unused) float pos_f = state[0]; float vel_f = state[1];

        // Density varies with altitude
        float rho = atmo_.get_density(state[0]);

        // rk4 iteration
        rk4_kp1 = rk4_step(state, dt, rho);

        state = rk4_kp1;
    }
    return state;
}