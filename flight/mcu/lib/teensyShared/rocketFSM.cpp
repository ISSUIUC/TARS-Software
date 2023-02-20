#include "rocketFSM.h"

TimerFSM timer_fsm;
HistoryBufferFSM<50> history_buffer_fsm_50;
HistoryBufferFSM<6> history_buffer_fsm_6;
KalmanFSM kalman_fsm;

FSMCollection<4> fsmCollection{&timer_fsm, &history_buffer_fsm_50, &history_buffer_fsm_6, &kalman_fsm};

RocketFSMBase& getActiveFSM() { return timer_fsm; }