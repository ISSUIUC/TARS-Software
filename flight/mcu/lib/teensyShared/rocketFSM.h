
#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

// #include "sensors.h"
#include <ChRt.h>
#include <stdint.h>

// forward declare pointers
struct pointers;

/**
 * @brief Labels for each FSM state
 */
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
    rocketFSM(pointers*);

    void tickFSM();

   private:
    pointers* pointer_struct;

    float* linear_acceleration_ptr_;
    FSM_State* rocket_state_ptr_;
    systime_t* launch_time_ptr_;
    sysinterval_t* burn_timer_ptr_;
    systime_t* burnout_time_ptr_;
    sysinterval_t* coast_timer_ptr_;
};

/**
 * @brief Struct to store event timestamps and timers since each event happened
 *
 * Times stored as ChibiOS systime_t and timers stored as sysinterval_t. Both of
 * these are typedefs for the C++ uint32_t type, but are good for readability
 * and clarity.
 */
struct fsm_struct {
    systime_t launch_time;  // First time acceleration above threshold is
                            // detected
    sysinterval_t burn_timer;

    systime_t burnout_time;
    sysinterval_t coast_timer;

    systime_t apogee_time;
    sysinterval_t apogee_timer;

    systime_t drogue_time;
    sysinterval_t drogue_timer;

    systime_t main_time;
    sysinterval_t main_timer;

    systime_t landing_time;
    sysinterval_t landing_timer;
};

#endif
