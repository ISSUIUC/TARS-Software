
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
     * @brief Labels for each FSM state. Contains intermediary states (eg: Launch Detect) along with actual states (eg: Boost)
     */
    enum class FSM_State {
        STATE_INIT,
        STATE_IDLE,
        STATE_LAUNCH_DETECT,
        STATE_BOOST,
        STATE_BURNOUT_DETECT,
        STATE_COAST_PREGNC,
        STATE_COAST_GNC,
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

    virtual void tickFSM() = 0;

    FSM_State getFSMState() const { return rocket_state_; }

   protected:
    FSM_State rocket_state_ = FSM_State::STATE_INIT;
};

/**
 * @brief Structure for all values related to rocket state
 * 
 * Contains the timestamp for FSM data along with an array of FSM states for each FSM in the flight code
 *
 */
template <size_t count>
struct rocketStateData {
    RocketFSM::FSM_State rocketStates[count];
    systime_t timeStamp_RS = 0;

    rocketStateData() : rocketStates() {
        for (size_t i = 0; i < count; i++) {
            rocketStates[i] = RocketFSM::FSM_State::STATE_INIT;
        }
    }
};

#endif
