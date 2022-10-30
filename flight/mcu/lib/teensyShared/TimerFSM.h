#pragma once

#include "RocketFSMBase.h"
#include "ChRt.h"

class TimerFSM : public RocketFSMBase {
public:
    void tickFSM() override;

private:
    systime_t launch_time_ = 0;
    sysinterval_t burn_timer_ = 0;
    systime_t burnout_time_ = 0;
    sysinterval_t coast_timer_ = 0;
};
