#pragma once

#include "mcu_main/finite-state-machines/FSMCollection.h"
#include "mcu_main/finite-state-machines/HistoryBufferFSM.h"
#include "mcu_main/finite-state-machines/KalmanFSM.h"
#include "mcu_main/finite-state-machines/ModularFSM.h"
#include "mcu_main/finite-state-machines/TimerFSM.h"
#include "mcu_main/finite-state-machines/ModularFSM.h"

extern TimerFSM timer_fsm;
extern ModularFSM modular_fsm;
extern HistoryBufferFSM<6> history_buffer_fsm_6;
extern KalmanFSM kalman_fsm;

// Pass array of FSMs to FSMCollection along with number of FSMs in use
extern FSMCollection<4> fsmCollection;

RocketFSMBase& getActiveFSM();
