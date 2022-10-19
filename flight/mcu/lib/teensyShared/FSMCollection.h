#pragma once
#include <ChRt.h>

#include "rocketFSM.h"

/**
 * @brief Manager for all FSMs in the rocket
 *
 * @param FSMs Array of pointers to all FSMs which implement RocketFSM
 * @param count Number of FSMs in FSMs array
 */

class FSMCollection {
   public:
    FSMCollection(RocketFSM** FSMs, size_t count)
        : FSMs_(FSMs), count_(count) {}

    void tick() {
        // tick all FSMs
        for (size_t i = 0; i < count_; i++) {
            FSMs_[i]->tickFSM();
        }
    }

    void getStates(rocketStateData* out) {
        // update FSM states and timestamps
        systime_t time = chVTGetSystemTime();
        for (size_t i = 0; i < count_; i++) {
            out[i].rocketState = FSMs_[i]->getFSMState();
            out[i].timeStamp_RS = time;
        }
    }

   private:
    RocketFSM** FSMs_;  // array of pointers to FSMs
    size_t count_;      // number of FSMs
};