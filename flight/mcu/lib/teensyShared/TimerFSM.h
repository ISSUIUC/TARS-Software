#pragma once

#include "rocketFSM.h"

class TimerFSM : public RocketFSM {
   public:
    TimerFSM(pointers*);
    virtual void tickFSM() override;

   private:
    pointers* pointer_struct;

    float* linear_acceleration_ptr_;
    systime_t launch_time_;
    sysinterval_t burn_timer_;
    systime_t burnout_time_;
    sysinterval_t coast_timer_;
};
