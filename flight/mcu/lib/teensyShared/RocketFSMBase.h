
#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

#include "packet.h"

class RocketFSMBase {
   public:
    virtual void tickFSM() = 0;

    FSM_State getFSMState() const { return rocket_state_; }

   protected:
    FSM_State rocket_state_ = FSM_State::STATE_INIT;
};

#endif
