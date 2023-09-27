#pragma once

#include "mcu_main/finite-state-machines/RocketFSMBase.h"
#include "mcu_main/Rt.h"

class KalmanFSM : public RocketFSMBase {
   public:
    KalmanFSM() = default;

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

    double getAltitudeAverage(size_t start, size_t len);
    double getSecondDerivativeAltitudeAverage(size_t start, size_t len);
    double getAccelerationAverage(size_t start, size_t len);
};
