#pragma once

#include "TimerFSM.h"
#include "HistoryBufferFSM.h"
#include "KalmanFSM.h"
#include "FSMCollection.h"

extern TimerFSM timer_fsm;
extern HistoryBufferFSM<50> history_buffer_fsm_50;
extern HistoryBufferFSM<6> history_buffer_fsm_6;
extern KalmanFSM kalman_fsm;

// Pass array of FSMs to FSMCollection along with number of FSMs in use
extern FSMCollection<4> fsmCollection;

RocketFSMBase& getActiveFSM();
