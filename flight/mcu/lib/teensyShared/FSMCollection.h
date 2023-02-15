#pragma once

#include "ChRt.h"
#include <initializer_list>  // this is here to make initializing the FSMCollection a lot more convenient

#include "RocketFSMBase.h"

/**
 * @brief Manager for all FSMs in the rocket
 *
 * @param FSMs Array of pointers to all FSMs which implement RocketFSMBase
 * @param count Number of FSMs in FSMs array
 */

template <size_t count>
class FSMCollection {
   public:
    FSMCollection(std::initializer_list<RocketFSMBase*> fsms) : FSMs_{} {
        size_t i = 0;
        for (auto& item : fsms) {
            if (i >= count) {
                break;
            }
            FSMs_[i++] = item;
        }
    }

    void tick() {
        // tick all FSMs
        for (size_t i = 0; i < count; i++) {
            FSMs_[i]->tickFSM();
        }
    }

    rocketStateData<count> getStates() {
        rocketStateData<count> states;
        // refresh FSM states and timestamps
        systime_t time = chVTGetSystemTime();
        for (size_t i = 0; i < count; i++) {
            states.rocketStates[i] = FSMs_[i]->getFSMState();
            states.timestamp = time;
        }
        return states;
    }

   private:
    RocketFSMBase* FSMs_[count];
};