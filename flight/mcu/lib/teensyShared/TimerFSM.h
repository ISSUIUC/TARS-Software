#pragma once

#include "rocketFSM.h"

class TimerFSM : public RocketFSM {
   public:
    TimerFSM(pointers*);
    virtual void tickFSM() override;

   private:
    pointers* pointer_struct;


    /**
     * @brief Timestamps and timers used to govern FSM logic
     * 
     * Time is the timestamp within which the event occurs
     * Timer is the amount time spent in each state and calculated based on 
     * the timestamps compared to the current time
    */
    float* linear_acceleration_ptr_;
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
};
