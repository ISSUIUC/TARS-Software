#pragma once
#include <ChRt.h>

#include <initializer_list>

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
    FSMCollection(std::initializer_list<RocketFSMBase*> fsms) {
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
        rocketStateData<count> ret;
        // refresh FSM states and timestamps
        systime_t time = chVTGetSystemTime();
        for (size_t i = 0; i < count; i++) {
            ret.rocketStates[i] = FSMs_[i]->getFSMState();
            ret.timestamp = time;
        }
        return ret;
    }

   private:
    RocketFSMBase* FSMs_[count]{};
};