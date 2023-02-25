#include "mcu_main/finite-state-machines/rocketFSM.h"

TimerFSM timer_fsm;
HistoryBufferFSM<50> history_buffer_fsm_50;
HistoryBufferFSM<6> history_buffer_fsm_6;
KalmanFSM kalman_fsm;
ModularFSM modular_fsm;

FSMCollection<5> fsmCollection{&timer_fsm, &modular_fsm, &history_buffer_fsm_50, &history_buffer_fsm_6, &kalman_fsm};

RocketFSMBase& getActiveFSM() { return timer_fsm; }