#pragma once

#include "RocketFSMBase.h"

class TimerFSM : public RocketFSMBase {
public:
    TimerFSM() = default;

    void tickFSM() override;

private:
    systime_t launch_time_ = 0;
    sysinterval_t burn_timer_ = 0;
    systime_t burnout_time_ = 0;
    sysinterval_t coast_timer_ = 0;

    systime_t apogee_time_ = 0;
    sysinterval_t apogee_timer_ = 0;

    systime_t drogue_time_ = 0;
    sysinterval_t drogue_timer_ = 0;

    systime_t main_time_ = 0;
    sysinterval_t main_timer_ = 0;

    systime_t landing_time_ = 0;
    sysinterval_t landing_timer = 0;
};
