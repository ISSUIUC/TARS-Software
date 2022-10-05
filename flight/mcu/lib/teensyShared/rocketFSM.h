
#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

// #include "sensors.h"
#include <stdint.h>

// forward declare pointers
struct pointers;

// Enum for FSM States
enum FSM_State {
    STATE_INIT,
    STATE_IDLE,
    STATE_LAUNCH_DETECT,
    STATE_BOOST,
    STATE_BURNOUT_DETECT,
    STATE_COAST,
    STATE_APOGEE_DETECT,
    STATE_APOGEE,
    STATE_DROGUE_DETECT,
    STATE_DROGUE,
    STATE_MAIN_DETECT,
    STATE_MAIN,
    STATE_LANDED_DETECT,
    STATE_LANDED,
    STATE_ABORT
};
class rocketFSM {
   public:
    rocketFSM(pointers *);

    void tickFSM();

   private:
   
    pointers *pointer_struct;
    
    float* linear_acceleration_ptr_;
    FSM_State* rocket_state_ptr_;
    uint32_t* launch_time_ptr_;
    uint32_t* burn_timer_ptr_;
    uint32_t* burnout_time_ptr_;
    uint32_t* coast_timer_ptr_;
    
};


// Stores time the rocket state is reached as well as timers used to confirm
// measurements
struct fsm_struct {
    uint32_t launch_time;  // First time acceleration above threshold is
                           // detected
    uint32_t burn_timer;

    uint32_t burnout_time;
    uint32_t coast_timer;

    uint32_t apogee_timer;
    uint32_t apogee_time;

    uint32_t drogue_time;
    uint32_t drogue_timer;

    uint32_t main_time;
    uint32_t main_timer;

    uint32_t landing_time;
    uint32_t landing_timer;
};

#endif
