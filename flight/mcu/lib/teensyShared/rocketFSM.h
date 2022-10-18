
#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

// #include "sensors.h"
#include <ChRt.h>
#include <stdint.h>

// forward declare pointers
struct pointers;

class RocketFSM {
   public:
   /**
     * @brief Labels for each FSM state
     */
    enum class FSM_State {
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

    RocketFSM(pointers*);

    void tickFSM();

    FSM_State getFSMState() const {
        return rocket_state_;
    }

    

   private:
    pointers* pointer_struct;

    float* linear_acceleration_ptr_;
    FSM_State rocket_state_;
    systime_t launch_time_;
    sysinterval_t burn_timer_;
    systime_t burnout_time_;
    sysinterval_t coast_timer_;
};

#endif
