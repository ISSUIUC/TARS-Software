#pragma once
#include "rocketFSM.h"
#include <ChRt.h>

class FSMCollection {
    public:
        FSMCollection(RocketFSM** FSMs, size_t count): FSMs_(FSMs), count_(count){

        }

        void tick(){
            for(size_t i = 0; i < count_; i++){
                FSMs_[i]->tickFSM();
            }
        }

        void getStates(rocketStateData* out){
            systime_t time = chVTGetSystemTime();
            for(size_t i = 0; i < count_; i++){
                out[i].rocketState = FSMs_[i]->getFSMState();
                out[i].timeStamp_RS = time;
            }
        }
    
    private:
        RocketFSM** FSMs_;
        size_t count_;
};