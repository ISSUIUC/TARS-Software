#pragma once
#include <ChRt.h>

#include "rocketFSM.h"

/**
 * @brief Manager for all FSMs in the rocket
 *
 * @param FSMs Array of pointers to all FSMs which implement RocketFSM
 * @param count Number of FSMs in FSMs array
 */

template <size_t count>
class FSMCollection {
   public:
    FSMCollection(RocketFSM** FSMs) : FSMs_(FSMs) {}

    void tick() {
        // tick all FSMs
        for (size_t i = 0; i < count; i++) {
            FSMs_[i]->tickFSM();
        }
    }

    void getStates(rocketStateData<count>& out) {
        // update FSM states and timestamps
        systime_t time = chVTGetSystemTime();
        for (size_t i = 0; i < count; i++) {
            out.rocketStates[i] = FSMs_[i]->getFSMState();
            out.timeStamp_RS = time;
        }
    }

   private:
    RocketFSM** FSMs_;  // array of pointers to FSMs
};