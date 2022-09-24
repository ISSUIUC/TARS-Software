#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include "dataLog.h"
#include "acShared.h"

// the following table should be generated by beep_fsm_gen.py
/* ======= AUTOGENERATED ======= */

enum BuzzerState {
    INITIAL,
    BUZZ_IDLE_STATE_0,
    BUZZ_IDLE_STATE_1,
    BUZZ_IDLE_STATE_2,
    BUZZ_IDLE_STATE_3,
    BUZZ_IDLE_STATE_4,
    BUZZ_IDLE_STATE_5,
    BUZZ_IDLE_STATE_6,
    BUZZ_IDLE_STATE_7,
};

unsigned int BuzzerStates[9][3] = {
        /* INITIAL*/ {0, 0, INITIAL},
        /* BUZZ_IDLE_STATE_0 */ { 1000, 250, BUZZ_IDLE_STATE_1 },
        /* BUZZ_IDLE_STATE_1 */ { 0, 250, BUZZ_IDLE_STATE_2 },
        /* BUZZ_IDLE_STATE_2 */ { 500, 750, BUZZ_IDLE_STATE_3 },
        /* BUZZ_IDLE_STATE_3 */ { 0, 500, BUZZ_IDLE_STATE_4 },
        /* BUZZ_IDLE_STATE_4 */ { 500, 750, BUZZ_IDLE_STATE_5 },
        /* BUZZ_IDLE_STATE_5 */ { 0, 500, BUZZ_IDLE_STATE_6 },
        /* BUZZ_IDLE_STATE_6 */ { 500, 750, BUZZ_IDLE_STATE_7 },
        /* BUZZ_IDLE_STATE_7 */ { 0, 500, INITIAL },
};

/* ======= END AUTOGENERATED ======= */

class BuzzerController {
public:
    explicit BuzzerController(pointers *);

    void tickBuzzer();
    void setBuzzerState(BuzzerState state);

private:
    BuzzerState curr_state;
    unsigned long time_since_state_start;

    pointers* rocket_state;
    FSM_State last_rocket_fsm_state;
};


#endif