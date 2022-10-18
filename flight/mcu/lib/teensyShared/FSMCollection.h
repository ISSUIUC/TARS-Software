#pragma once
#include "rocketFSM.h"
#include "dataLog.h"
#include<ChRt.h>

class FSMCollection {
    public:
        FSMCollection(RocketFSM*[] FSMs, size_t count): FSMs(FSMs), count(count){

        }

        void tick(){
            for(size_t i = 0; i < count; i++){
                FSMs[i]->tickFSM();
            }
        }

        void getStates(rocketStateData* out){
            systime_t time = chvt
            for(size_t i = 0; i < count; i++){
                out[i].rocketState = FSMs[i]->getState();
                out[i].timeStamp_RS = chVTGetSystemTime();
            }
        }
    
    private:
        RocketFSM** FSMs;
        size_t count;
}