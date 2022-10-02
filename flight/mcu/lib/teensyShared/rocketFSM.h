
#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

#include "sensors.h"

class rocketFSM {
   public:
    rocketFSM(pointers *);

    void tickFSM();

   private:
    pointers *pointer_struct;
    float linear_acceleration;
    FSM_State rocket_state;
    uint32_t launch_time;
    uint32_t burn_timer;
    uint32_t burnout_time;
    uint32_t coast_timer;


};

#endif
