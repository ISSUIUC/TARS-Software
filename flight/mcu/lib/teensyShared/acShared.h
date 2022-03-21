#ifndef ACSHARED_H
#define ACSHARED_H

#include <stdint.h>

// data structure for FSM States

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

// stores time the rocket state is reached as well as timers used to confirm
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
