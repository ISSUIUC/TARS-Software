#pragma once

#include "RocketFSMBase.h"
#include "dataLog.h"

class KalmanFSM : public RocketFSMBase {
   public:
    void tickFSM() override;

   private:
    systime_t launch_time_{};
    sysinterval_t burn_timer_{};
    systime_t burnout_time_{};
    sysinterval_t coast_timer_{};

    systime_t apogee_time_{};
    sysinterval_t apogee_timer_{};

    systime_t drogue_time_{};
    sysinterval_t drogue_timer_{};

    systime_t main_time_{};
    sysinterval_t main_timer_{};

    systime_t landing_time_{};
    sysinterval_t landing_timer{};
};
