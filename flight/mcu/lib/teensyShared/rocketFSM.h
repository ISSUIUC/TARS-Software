#ifndef MCU_ROCKETFSM_H
#define MCU_ROCKETFSM_H

#include "TimerFSM.h"
#include "HistoryBufferFSM50.h"
#include "HistoryBufferFSM6.h"
#include "KalmanFSM.h"
#include "FSMCollection.h"

extern TimerFSM timer_fsm;
extern HistoryBufferFSM50 history_buffer_fsm_50;
extern HistoryBufferFSM6 history_buffer_fsm_6;
extern KalmanFSM kalman_fsm;

// Pass array of FSMs to FSMCollection along with number of FSMs in use
extern FSMCollection<4> fsmCollection;

RocketFSMBase& getActiveFSM();

#endif //MCU_ROCKETFSM_H
