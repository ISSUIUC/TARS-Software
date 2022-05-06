#include <iostream>
#include <array>
#include <chrono>
#include <cmath>
#include "rk4.h"

using namespace std::chrono;

using std::array;

rk4::rk4() {
        Atmosphere atmo;
        atmo_ = atmo;
    }

float rk4::cd(float alt, float vel) {
    float mach = vel/(atmo_.get_speed_of_sound(alt));
    double cd = 0;
    for(int i = 0; i < 151; i++) {
        cd += poly[i]*std::pow(mach, 150-i);
    }
    return float(cd);
}

array<float, 2> rk4::accel(array<float, 2> u, float rho) {
    float r1 = u[0];
    float v1 = u[1];
    
    // Approximation - use the area of a circle for reference area
    float Sref_a = .007854;
    // Cd_total = rasaero.drag_lookup_1dof(pos_f,vel_f,RASaero,dic["CD"])
    float Cd_total = cd(r1, v1);

    float F_a = -((rho* (v1*v1) * Sref_a * Cd_total) / 2);
    float accel_a = F_a/21; // Acceleration due to Aerodynamic Forces
    float accel_f = accel_a - 9.81;

    array<float, 2> f1 {v1, accel_f};

    return f1;
}

array<float, 2> rk4::rk4_step(array<float, 2> state, float dt, float rho) {
    // rk4 iteration 
    array<float, 2> y1 = accel(state, rho);

    array<float, 2> u1 = {(float)(state[0] + .5*dt*y1[0]), (float)(state[1] + .5*dt*y1[1])};
    array<float, 2> u2 = {(float)(state[0] + .5*dt*y2[0]), (float)(state[1] + .5*dt*y2[1])};
    array<float, 2> u3 = {(float)(state[0] + dt*y3[0]), (float)(state[1] + dt*y3[1])};

    array<float, 2> y2 = accel(u1, rho);
    array<float, 2> y3 = accel(u2, rho);
    array<float, 2> y4 = accel(u3, rho);

    array<float, 2> temp = {state[0] + (dt*(y1[0] + 2*y2[0] + 2*y3[0] + y4[0]))/6, 
                            state[1] + (dt*(y1[1] + 2*y2[1] + 2*y3[1] + y4[1]))/6};

    // rk4_kp1 = state + dt*(y1 + 2*y2 + 2*y3 + y4)/6;
    return temp;
}

array<float, 2> rk4::sim_apogee(array<float, 2> state, float dt) {
    
    // Approximation - use the area of a circle for reference area
    float Sref_a = .007854;
    
    while (state[1] > 0) {
        
        // Define initial flap length at start of control time
        float l = 0;
        
        // grabbing the current states
        float pos_f = state[0];
        float vel_f = state[1];
        
        // Density varies with altitude
        float rho = atmo_.get_density(state[0]);
        
        
        
        // rk4 iteration 
        rk4_kp1 = rk4_step(state, dt, rho);
        
        state = rk4_kp1;
    }
    return state;
}