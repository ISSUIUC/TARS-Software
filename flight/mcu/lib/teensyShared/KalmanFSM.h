#pragma once

#include "dataLog.h"
#include "rocketFSM.h"

class KalmanFSM : public RocketFSM {
   public:
    KalmanFSM(pointers*);
    virtual void tickFSM() override;

   private:
    pointers* pointer_struct;

    stateData* gnc_state_ptr_;

    systime_t launch_time_;
    sysinterval_t burn_timer_;
    systime_t burnout_time_;
    sysinterval_t coast_timer_;

    systime_t apogee_time_;
    sysinterval_t apogee_timer_;

    systime_t drogue_time_;
    sysinterval_t drogue_timer_;

    systime_t main_time_;
    sysinterval_t main_timer_;

    systime_t landing_time_;
    sysinterval_t landing_timer;

    HistoryBuffer<6>* altitude_history_ptr_;
    HistoryBuffer<6>* IMU_acceleration_history_ptr_;
};
