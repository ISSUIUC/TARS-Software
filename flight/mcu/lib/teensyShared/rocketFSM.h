/**
 * @file        rocketFSM.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       temp
 *
 * temp
 * 
 * 
 * 
 *
 */

#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

#include "sensors.h"

class rocketFSM {
   public:
    rocketFSM(pointers *);

    void tickFSM();

   private:
    pointers *pointer_struct;
};

#endif
