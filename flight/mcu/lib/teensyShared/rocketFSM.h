
#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

// #include "sensors.h"
#include <ChRt.h>
#include <stdint.h>
#include <map>

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

    std::map<int, String> state_map = { {0, "STATE_INIT"}, {1, "STATE_IDLE"},
    {2, "STATE_LAUNCH_DETECT"},
    {3, "STATE_BOOST"},
    {4, "STATE_BURNOUT_DETECT"},
    {5, "STATE_COAST_PREGNC"},
    {6, "STATE_COAST_GNC"},
    {7, "STATE_APOGEE_DETECT"},
    {8, "STATE_APOGEE"},
    {9, "STATE_DROGUE_DETECT"},
    {10, "STATE_DROGUE"},
    {11,"STATE_MAIN_DETECT"},
    {12, "STATE_MAIN"},
    {13, "STATE_LANDED_DETECT"},
    {14, "STATE_LANDED"},
    {15, "STATE_ABORT"}};

    virtual void tickFSM() = 0;

    FSM_State getFSMState() const { return rocket_state_; }

   protected:
    FSM_State rocket_state_ = FSM_State::STATE_INIT;
};

/**
 * @brief Structure for all values related to rocket state
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
