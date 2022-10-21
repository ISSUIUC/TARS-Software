#pragma once

#include "rocketFSM.h"
#include "HistoryBuffer.h"

class HistoryBufferFSM : public RocketFSM {
   public:
    HistoryBufferFSM(pointers*);
    virtual void tickFSM() override;

   private:
    pointers* pointer_struct;

    float* linear_acceleration_ptr_;
    systime_t launch_time_;
    sysinterval_t burn_timer_;
    systime_t burnout_time_;
    sysinterval_t coast_timer_;
    HistoryBuffer* altitude_history_ptr_;
    HistoryBuffer* IMU_acceleration_history_ptr_;
};
