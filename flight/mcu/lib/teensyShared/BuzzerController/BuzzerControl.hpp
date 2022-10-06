#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include "dataLog.h"
#include "acShared.h"

// the following table should be generated by beep_fsm_gen.py
// TODO this doesn't need a full fsm, we can get away with a struct of a sequence and a "should_loop" parameter
// TODO it would make the code a lot simpler and more comprehensible
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
    BUZZ_BOOST_STATE_0,
    BUZZ_BOOST_STATE_1,
    BUZZ_BOOST_STATE_2,
    BUZZ_BOOST_STATE_3,
    BUZZ_BOOST_STATE_4,
    BUZZ_BOOST_STATE_5,
    BUZZ_BOOST_STATE_6,
    BUZZ_BOOST_STATE_7,
    BUZZ_COAST_STATE_0,
    BUZZ_COAST_STATE_1,
    BUZZ_COAST_STATE_2,
    BUZZ_COAST_STATE_3,
    BUZZ_COAST_STATE_4,
    BUZZ_COAST_STATE_5,
    BUZZ_COAST_STATE_6,
    BUZZ_COAST_STATE_7,
    BUZZ_APOGEE_STATE_0,
    BUZZ_APOGEE_STATE_1,
    BUZZ_APOGEE_STATE_2,
    BUZZ_APOGEE_STATE_3,
    BUZZ_APOGEE_STATE_4,
    BUZZ_APOGEE_STATE_5,
    BUZZ_APOGEE_STATE_6,
    BUZZ_APOGEE_STATE_7,
    BUZZ_DROGUE_STATE_0,
    BUZZ_DROGUE_STATE_1,
    BUZZ_DROGUE_STATE_2,
    BUZZ_DROGUE_STATE_3,
    BUZZ_DROGUE_STATE_4,
    BUZZ_DROGUE_STATE_5,
    BUZZ_DROGUE_STATE_6,
    BUZZ_DROGUE_STATE_7,
    BUZZ_MAIN_STATE_0,
    BUZZ_MAIN_STATE_1,
    BUZZ_MAIN_STATE_2,
    BUZZ_MAIN_STATE_3,
    BUZZ_MAIN_STATE_4,
    BUZZ_MAIN_STATE_5,
    BUZZ_MAIN_STATE_6,
    BUZZ_MAIN_STATE_7,
    BUZZ_LANDED_STATE_0,
    BUZZ_LANDED_STATE_1,
    BUZZ_LANDED_STATE_2,
    BUZZ_LANDED_STATE_3,
    BUZZ_LANDED_STATE_4,
    BUZZ_LANDED_STATE_5,
    BUZZ_LANDED_STATE_6,
    BUZZ_LANDED_STATE_7,
    BUZZ_ABORT_STATE_0,
    BUZZ_ABORT_STATE_1,
    BUZZ_ABORT_STATE_2,
    BUZZ_ABORT_STATE_3,
    BUZZ_ABORT_STATE_4,
    BUZZ_ABORT_STATE_5,
    BUZZ_ABORT_STATE_6,
    BUZZ_ABORT_STATE_7,
    BATTERY_ZERO_STATE_0,
    BATTERY_ZERO_STATE_1,
    BATTERY_ZERO_STATE_2,
    BATTERY_ZERO_STATE_3,
    BATTERY_ZERO_STATE_4,
    BATTERY_ZERO_STATE_5,
    BATTERY_ZERO_STATE_6,
    BATTERY_ZERO_STATE_7,
    BATTERY_ZERO_STATE_8,
    BATTERY_ZERO_STATE_9,
    BATTERY_ZERO_STATE_10,
    BATTERY_ZERO_STATE_11,
    BATTERY_ZERO_STATE_12,
    BATTERY_ZERO_STATE_13,
    BATTERY_ZERO_STATE_14,
    BATTERY_ZERO_STATE_15,
    BATTERY_ZERO_STATE_16,
    BATTERY_ZERO_STATE_17,
    BATTERY_ZERO_STATE_18,
    BATTERY_ZERO_STATE_19,
    BATTERY_ZERO_STATE_20,
    BATTERY_ZERO_STATE_21,
    BATTERY_ONE_STATE_0,
    BATTERY_ONE_STATE_1,
    BATTERY_ONE_STATE_2,
    BATTERY_ONE_STATE_3,
    BATTERY_TWO_STATE_0,
    BATTERY_TWO_STATE_1,
    BATTERY_TWO_STATE_2,
    BATTERY_TWO_STATE_3,
    BATTERY_TWO_STATE_4,
    BATTERY_TWO_STATE_5,
    BATTERY_THREE_STATE_0,
    BATTERY_THREE_STATE_1,
    BATTERY_THREE_STATE_2,
    BATTERY_THREE_STATE_3,
    BATTERY_THREE_STATE_4,
    BATTERY_THREE_STATE_5,
    BATTERY_THREE_STATE_6,
    BATTERY_THREE_STATE_7,
    BATTERY_FOUR_STATE_0,
    BATTERY_FOUR_STATE_1,
    BATTERY_FOUR_STATE_2,
    BATTERY_FOUR_STATE_3,
    BATTERY_FOUR_STATE_4,
    BATTERY_FOUR_STATE_5,
    BATTERY_FOUR_STATE_6,
    BATTERY_FOUR_STATE_7,
    BATTERY_FOUR_STATE_8,
    BATTERY_FOUR_STATE_9,
    BATTERY_FIVE_STATE_0,
    BATTERY_FIVE_STATE_1,
    BATTERY_FIVE_STATE_2,
    BATTERY_FIVE_STATE_3,
    BATTERY_FIVE_STATE_4,
    BATTERY_FIVE_STATE_5,
    BATTERY_FIVE_STATE_6,
    BATTERY_FIVE_STATE_7,
    BATTERY_FIVE_STATE_8,
    BATTERY_FIVE_STATE_9,
    BATTERY_FIVE_STATE_10,
    BATTERY_FIVE_STATE_11,
    BATTERY_SIX_STATE_0,
    BATTERY_SIX_STATE_1,
    BATTERY_SIX_STATE_2,
    BATTERY_SIX_STATE_3,
    BATTERY_SIX_STATE_4,
    BATTERY_SIX_STATE_5,
    BATTERY_SIX_STATE_6,
    BATTERY_SIX_STATE_7,
    BATTERY_SIX_STATE_8,
    BATTERY_SIX_STATE_9,
    BATTERY_SIX_STATE_10,
    BATTERY_SIX_STATE_11,
    BATTERY_SIX_STATE_12,
    BATTERY_SIX_STATE_13,
    BATTERY_SIX_STATE_14,
    BATTERY_SIX_STATE_15,
    BATTERY_SEVEN_STATE_0,
    BATTERY_SEVEN_STATE_1,
    BATTERY_SEVEN_STATE_2,
    BATTERY_SEVEN_STATE_3,
    BATTERY_SEVEN_STATE_4,
    BATTERY_SEVEN_STATE_5,
    BATTERY_SEVEN_STATE_6,
    BATTERY_SEVEN_STATE_7,
    BATTERY_SEVEN_STATE_8,
    BATTERY_SEVEN_STATE_9,
    BATTERY_SEVEN_STATE_10,
    BATTERY_SEVEN_STATE_11,
    BATTERY_SEVEN_STATE_12,
    BATTERY_SEVEN_STATE_13,
    BATTERY_SEVEN_STATE_14,
    BATTERY_SEVEN_STATE_15,
    BATTERY_SEVEN_STATE_16,
    BATTERY_SEVEN_STATE_17,
    BATTERY_SEVEN_STATE_18,
    BATTERY_SEVEN_STATE_19,
    BATTERY_EIGHT_STATE_0,
    BATTERY_EIGHT_STATE_1,
    BATTERY_EIGHT_STATE_2,
    BATTERY_EIGHT_STATE_3,
    BATTERY_EIGHT_STATE_4,
    BATTERY_EIGHT_STATE_5,
    BATTERY_EIGHT_STATE_6,
    BATTERY_EIGHT_STATE_7,
    BATTERY_EIGHT_STATE_8,
    BATTERY_EIGHT_STATE_9,
    BATTERY_EIGHT_STATE_10,
    BATTERY_EIGHT_STATE_11,
    BATTERY_EIGHT_STATE_12,
    BATTERY_EIGHT_STATE_13,
    BATTERY_EIGHT_STATE_14,
    BATTERY_EIGHT_STATE_15,
    BATTERY_EIGHT_STATE_16,
    BATTERY_EIGHT_STATE_17,
    BATTERY_NINE_STATE_0,
    BATTERY_NINE_STATE_1,
    BATTERY_NINE_STATE_2,
    BATTERY_NINE_STATE_3,
    BATTERY_NINE_STATE_4,
    BATTERY_NINE_STATE_5,
    BATTERY_NINE_STATE_6,
    BATTERY_NINE_STATE_7,
    BATTERY_NINE_STATE_8,
    BATTERY_NINE_STATE_9,
    BATTERY_NINE_STATE_10,
    BATTERY_NINE_STATE_11,
    BATTERY_NINE_STATE_12,
    BATTERY_NINE_STATE_13,
    BATTERY_NINE_STATE_14,
    BATTERY_NINE_STATE_15,
    BATTERY_NINE_STATE_16,
    BATTERY_NINE_STATE_17,
    BATTERY_NINE_STATE_18,
    BATTERY_NINE_STATE_19,
};

unsigned int BuzzerStates[201][3] = {
    /* INITIAL*/ {0, 0, INITIAL},
    /* BUZZ_IDLE_STATE_0 */ { 1000, 250, BUZZ_IDLE_STATE_1 },
    /* BUZZ_IDLE_STATE_1 */ { 0, 500, BUZZ_IDLE_STATE_2 },
    /* BUZZ_IDLE_STATE_2 */ { 500, 250, BUZZ_IDLE_STATE_3 },
    /* BUZZ_IDLE_STATE_3 */ { 0, 500, BUZZ_IDLE_STATE_4 },
    /* BUZZ_IDLE_STATE_4 */ { 500, 250, BUZZ_IDLE_STATE_5 },
    /* BUZZ_IDLE_STATE_5 */ { 0, 500, BUZZ_IDLE_STATE_6 },
    /* BUZZ_IDLE_STATE_6 */ { 500, 750, BUZZ_IDLE_STATE_7 },
    /* BUZZ_IDLE_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_BOOST_STATE_0 */ { 1000, 250, BUZZ_BOOST_STATE_1 },
    /* BUZZ_BOOST_STATE_1 */ { 0, 500, BUZZ_BOOST_STATE_2 },
    /* BUZZ_BOOST_STATE_2 */ { 500, 250, BUZZ_BOOST_STATE_3 },
    /* BUZZ_BOOST_STATE_3 */ { 0, 500, BUZZ_BOOST_STATE_4 },
    /* BUZZ_BOOST_STATE_4 */ { 500, 750, BUZZ_BOOST_STATE_5 },
    /* BUZZ_BOOST_STATE_5 */ { 0, 500, BUZZ_BOOST_STATE_6 },
    /* BUZZ_BOOST_STATE_6 */ { 500, 250, BUZZ_BOOST_STATE_7 },
    /* BUZZ_BOOST_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_COAST_STATE_0 */ { 1000, 250, BUZZ_COAST_STATE_1 },
    /* BUZZ_COAST_STATE_1 */ { 0, 500, BUZZ_COAST_STATE_2 },
    /* BUZZ_COAST_STATE_2 */ { 500, 250, BUZZ_COAST_STATE_3 },
    /* BUZZ_COAST_STATE_3 */ { 0, 500, BUZZ_COAST_STATE_4 },
    /* BUZZ_COAST_STATE_4 */ { 500, 750, BUZZ_COAST_STATE_5 },
    /* BUZZ_COAST_STATE_5 */ { 0, 500, BUZZ_COAST_STATE_6 },
    /* BUZZ_COAST_STATE_6 */ { 500, 750, BUZZ_COAST_STATE_7 },
    /* BUZZ_COAST_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_APOGEE_STATE_0 */ { 1000, 250, BUZZ_APOGEE_STATE_1 },
    /* BUZZ_APOGEE_STATE_1 */ { 0, 500, BUZZ_APOGEE_STATE_2 },
    /* BUZZ_APOGEE_STATE_2 */ { 500, 750, BUZZ_APOGEE_STATE_3 },
    /* BUZZ_APOGEE_STATE_3 */ { 0, 500, BUZZ_APOGEE_STATE_4 },
    /* BUZZ_APOGEE_STATE_4 */ { 500, 250, BUZZ_APOGEE_STATE_5 },
    /* BUZZ_APOGEE_STATE_5 */ { 0, 500, BUZZ_APOGEE_STATE_6 },
    /* BUZZ_APOGEE_STATE_6 */ { 500, 250, BUZZ_APOGEE_STATE_7 },
    /* BUZZ_APOGEE_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_DROGUE_STATE_0 */ { 1000, 250, BUZZ_DROGUE_STATE_1 },
    /* BUZZ_DROGUE_STATE_1 */ { 0, 500, BUZZ_DROGUE_STATE_2 },
    /* BUZZ_DROGUE_STATE_2 */ { 500, 750, BUZZ_DROGUE_STATE_3 },
    /* BUZZ_DROGUE_STATE_3 */ { 0, 500, BUZZ_DROGUE_STATE_4 },
    /* BUZZ_DROGUE_STATE_4 */ { 500, 250, BUZZ_DROGUE_STATE_5 },
    /* BUZZ_DROGUE_STATE_5 */ { 0, 500, BUZZ_DROGUE_STATE_6 },
    /* BUZZ_DROGUE_STATE_6 */ { 500, 750, BUZZ_DROGUE_STATE_7 },
    /* BUZZ_DROGUE_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_MAIN_STATE_0 */ { 1000, 250, BUZZ_MAIN_STATE_1 },
    /* BUZZ_MAIN_STATE_1 */ { 0, 500, BUZZ_MAIN_STATE_2 },
    /* BUZZ_MAIN_STATE_2 */ { 500, 750, BUZZ_MAIN_STATE_3 },
    /* BUZZ_MAIN_STATE_3 */ { 0, 500, BUZZ_MAIN_STATE_4 },
    /* BUZZ_MAIN_STATE_4 */ { 500, 750, BUZZ_MAIN_STATE_5 },
    /* BUZZ_MAIN_STATE_5 */ { 0, 500, BUZZ_MAIN_STATE_6 },
    /* BUZZ_MAIN_STATE_6 */ { 500, 250, BUZZ_MAIN_STATE_7 },
    /* BUZZ_MAIN_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_LANDED_STATE_0 */ { 1000, 250, BUZZ_LANDED_STATE_1 },
    /* BUZZ_LANDED_STATE_1 */ { 0, 500, BUZZ_LANDED_STATE_2 },
    /* BUZZ_LANDED_STATE_2 */ { 500, 750, BUZZ_LANDED_STATE_3 },
    /* BUZZ_LANDED_STATE_3 */ { 0, 500, BUZZ_LANDED_STATE_4 },
    /* BUZZ_LANDED_STATE_4 */ { 500, 750, BUZZ_LANDED_STATE_5 },
    /* BUZZ_LANDED_STATE_5 */ { 0, 500, BUZZ_LANDED_STATE_6 },
    /* BUZZ_LANDED_STATE_6 */ { 500, 750, BUZZ_LANDED_STATE_7 },
    /* BUZZ_LANDED_STATE_7 */ { 0, 500, INITIAL },
    /* BUZZ_ABORT_STATE_0 */ { 1000, 750, BUZZ_ABORT_STATE_1 },
    /* BUZZ_ABORT_STATE_1 */ { 0, 500, BUZZ_ABORT_STATE_2 },
    /* BUZZ_ABORT_STATE_2 */ { 500, 250, BUZZ_ABORT_STATE_3 },
    /* BUZZ_ABORT_STATE_3 */ { 0, 500, BUZZ_ABORT_STATE_4 },
    /* BUZZ_ABORT_STATE_4 */ { 500, 250, BUZZ_ABORT_STATE_5 },
    /* BUZZ_ABORT_STATE_5 */ { 0, 500, BUZZ_ABORT_STATE_6 },
    /* BUZZ_ABORT_STATE_6 */ { 500, 250, BUZZ_ABORT_STATE_7 },
    /* BUZZ_ABORT_STATE_7 */ { 0, 500, INITIAL },
    /* BATTERY_ZERO_STATE_0 */ { 1000, 250, BATTERY_ZERO_STATE_1 },
    /* BATTERY_ZERO_STATE_1 */ { 0, 500, BATTERY_ZERO_STATE_2 },
    /* BATTERY_ZERO_STATE_2 */ { 1500, 750, BATTERY_ZERO_STATE_3 },
    /* BATTERY_ZERO_STATE_3 */ { 0, 500, BATTERY_ZERO_STATE_4 },
    /* BATTERY_ZERO_STATE_4 */ { 1500, 750, BATTERY_ZERO_STATE_5 },
    /* BATTERY_ZERO_STATE_5 */ { 0, 500, BATTERY_ZERO_STATE_6 },
    /* BATTERY_ZERO_STATE_6 */ { 1500, 750, BATTERY_ZERO_STATE_7 },
    /* BATTERY_ZERO_STATE_7 */ { 0, 500, BATTERY_ZERO_STATE_8 },
    /* BATTERY_ZERO_STATE_8 */ { 1500, 750, BATTERY_ZERO_STATE_9 },
    /* BATTERY_ZERO_STATE_9 */ { 0, 500, BATTERY_ZERO_STATE_10 },
    /* BATTERY_ZERO_STATE_10 */ { 1500, 750, BATTERY_ZERO_STATE_11 },
    /* BATTERY_ZERO_STATE_11 */ { 0, 500, BATTERY_ZERO_STATE_12 },
    /* BATTERY_ZERO_STATE_12 */ { 1500, 750, BATTERY_ZERO_STATE_13 },
    /* BATTERY_ZERO_STATE_13 */ { 0, 500, BATTERY_ZERO_STATE_14 },
    /* BATTERY_ZERO_STATE_14 */ { 1500, 750, BATTERY_ZERO_STATE_15 },
    /* BATTERY_ZERO_STATE_15 */ { 0, 500, BATTERY_ZERO_STATE_16 },
    /* BATTERY_ZERO_STATE_16 */ { 1500, 750, BATTERY_ZERO_STATE_17 },
    /* BATTERY_ZERO_STATE_17 */ { 0, 500, BATTERY_ZERO_STATE_18 },
    /* BATTERY_ZERO_STATE_18 */ { 1500, 750, BATTERY_ZERO_STATE_19 },
    /* BATTERY_ZERO_STATE_19 */ { 0, 500, BATTERY_ZERO_STATE_20 },
    /* BATTERY_ZERO_STATE_20 */ { 1500, 750, BATTERY_ZERO_STATE_21 },
    /* BATTERY_ZERO_STATE_21 */ { 0, 500, INITIAL },
    /* BATTERY_ONE_STATE_0 */ { 1000, 250, BATTERY_ONE_STATE_1 },
    /* BATTERY_ONE_STATE_1 */ { 0, 500, BATTERY_ONE_STATE_2 },
    /* BATTERY_ONE_STATE_2 */ { 1500, 750, BATTERY_ONE_STATE_3 },
    /* BATTERY_ONE_STATE_3 */ { 0, 500, INITIAL },
    /* BATTERY_TWO_STATE_0 */ { 1000, 250, BATTERY_TWO_STATE_1 },
    /* BATTERY_TWO_STATE_1 */ { 0, 500, BATTERY_TWO_STATE_2 },
    /* BATTERY_TWO_STATE_2 */ { 1500, 750, BATTERY_TWO_STATE_3 },
    /* BATTERY_TWO_STATE_3 */ { 0, 500, BATTERY_TWO_STATE_4 },
    /* BATTERY_TWO_STATE_4 */ { 1500, 750, BATTERY_TWO_STATE_5 },
    /* BATTERY_TWO_STATE_5 */ { 0, 500, INITIAL },
    /* BATTERY_THREE_STATE_0 */ { 1000, 250, BATTERY_THREE_STATE_1 },
    /* BATTERY_THREE_STATE_1 */ { 0, 500, BATTERY_THREE_STATE_2 },
    /* BATTERY_THREE_STATE_2 */ { 1500, 750, BATTERY_THREE_STATE_3 },
    /* BATTERY_THREE_STATE_3 */ { 0, 500, BATTERY_THREE_STATE_4 },
    /* BATTERY_THREE_STATE_4 */ { 1500, 750, BATTERY_THREE_STATE_5 },
    /* BATTERY_THREE_STATE_5 */ { 0, 500, BATTERY_THREE_STATE_6 },
    /* BATTERY_THREE_STATE_6 */ { 1500, 750, BATTERY_THREE_STATE_7 },
    /* BATTERY_THREE_STATE_7 */ { 0, 500, INITIAL },
    /* BATTERY_FOUR_STATE_0 */ { 1000, 250, BATTERY_FOUR_STATE_1 },
    /* BATTERY_FOUR_STATE_1 */ { 0, 500, BATTERY_FOUR_STATE_2 },
    /* BATTERY_FOUR_STATE_2 */ { 1500, 750, BATTERY_FOUR_STATE_3 },
    /* BATTERY_FOUR_STATE_3 */ { 0, 500, BATTERY_FOUR_STATE_4 },
    /* BATTERY_FOUR_STATE_4 */ { 1500, 750, BATTERY_FOUR_STATE_5 },
    /* BATTERY_FOUR_STATE_5 */ { 0, 500, BATTERY_FOUR_STATE_6 },
    /* BATTERY_FOUR_STATE_6 */ { 1500, 750, BATTERY_FOUR_STATE_7 },
    /* BATTERY_FOUR_STATE_7 */ { 0, 500, BATTERY_FOUR_STATE_8 },
    /* BATTERY_FOUR_STATE_8 */ { 1500, 750, BATTERY_FOUR_STATE_9 },
    /* BATTERY_FOUR_STATE_9 */ { 0, 500, INITIAL },
    /* BATTERY_FIVE_STATE_0 */ { 1000, 250, BATTERY_FIVE_STATE_1 },
    /* BATTERY_FIVE_STATE_1 */ { 0, 500, BATTERY_FIVE_STATE_2 },
    /* BATTERY_FIVE_STATE_2 */ { 1500, 750, BATTERY_FIVE_STATE_3 },
    /* BATTERY_FIVE_STATE_3 */ { 0, 500, BATTERY_FIVE_STATE_4 },
    /* BATTERY_FIVE_STATE_4 */ { 1500, 750, BATTERY_FIVE_STATE_5 },
    /* BATTERY_FIVE_STATE_5 */ { 0, 500, BATTERY_FIVE_STATE_6 },
    /* BATTERY_FIVE_STATE_6 */ { 1500, 750, BATTERY_FIVE_STATE_7 },
    /* BATTERY_FIVE_STATE_7 */ { 0, 500, BATTERY_FIVE_STATE_8 },
    /* BATTERY_FIVE_STATE_8 */ { 1500, 750, BATTERY_FIVE_STATE_9 },
    /* BATTERY_FIVE_STATE_9 */ { 0, 500, BATTERY_FIVE_STATE_10 },
    /* BATTERY_FIVE_STATE_10 */ { 1500, 750, BATTERY_FIVE_STATE_11 },
    /* BATTERY_FIVE_STATE_11 */ { 0, 500, INITIAL },
    /* BATTERY_SIX_STATE_0 */ { 1000, 250, BATTERY_SIX_STATE_1 },
    /* BATTERY_SIX_STATE_1 */ { 0, 500, BATTERY_SIX_STATE_2 },
    /* BATTERY_SIX_STATE_2 */ { 1000, 250, BATTERY_SIX_STATE_3 },
    /* BATTERY_SIX_STATE_3 */ { 0, 500, BATTERY_SIX_STATE_4 },
    /* BATTERY_SIX_STATE_4 */ { 1500, 750, BATTERY_SIX_STATE_5 },
    /* BATTERY_SIX_STATE_5 */ { 0, 500, BATTERY_SIX_STATE_6 },
    /* BATTERY_SIX_STATE_6 */ { 1500, 750, BATTERY_SIX_STATE_7 },
    /* BATTERY_SIX_STATE_7 */ { 0, 500, BATTERY_SIX_STATE_8 },
    /* BATTERY_SIX_STATE_8 */ { 1500, 750, BATTERY_SIX_STATE_9 },
    /* BATTERY_SIX_STATE_9 */ { 0, 500, BATTERY_SIX_STATE_10 },
    /* BATTERY_SIX_STATE_10 */ { 1500, 750, BATTERY_SIX_STATE_11 },
    /* BATTERY_SIX_STATE_11 */ { 0, 500, BATTERY_SIX_STATE_12 },
    /* BATTERY_SIX_STATE_12 */ { 1500, 750, BATTERY_SIX_STATE_13 },
    /* BATTERY_SIX_STATE_13 */ { 0, 500, BATTERY_SIX_STATE_14 },
    /* BATTERY_SIX_STATE_14 */ { 1500, 750, BATTERY_SIX_STATE_15 },
    /* BATTERY_SIX_STATE_15 */ { 0, 500, INITIAL },
    /* BATTERY_SEVEN_STATE_0 */ { 1000, 250, BATTERY_SEVEN_STATE_1 },
    /* BATTERY_SEVEN_STATE_1 */ { 0, 500, BATTERY_SEVEN_STATE_2 },
    /* BATTERY_SEVEN_STATE_2 */ { 1000, 250, BATTERY_SEVEN_STATE_3 },
    /* BATTERY_SEVEN_STATE_3 */ { 0, 500, BATTERY_SEVEN_STATE_4 },
    /* BATTERY_SEVEN_STATE_4 */ { 1000, 250, BATTERY_SEVEN_STATE_5 },
    /* BATTERY_SEVEN_STATE_5 */ { 0, 500, BATTERY_SEVEN_STATE_6 },
    /* BATTERY_SEVEN_STATE_6 */ { 1500, 750, BATTERY_SEVEN_STATE_7 },
    /* BATTERY_SEVEN_STATE_7 */ { 0, 500, BATTERY_SEVEN_STATE_8 },
    /* BATTERY_SEVEN_STATE_8 */ { 1500, 750, BATTERY_SEVEN_STATE_9 },
    /* BATTERY_SEVEN_STATE_9 */ { 0, 500, BATTERY_SEVEN_STATE_10 },
    /* BATTERY_SEVEN_STATE_10 */ { 1500, 750, BATTERY_SEVEN_STATE_11 },
    /* BATTERY_SEVEN_STATE_11 */ { 0, 500, BATTERY_SEVEN_STATE_12 },
    /* BATTERY_SEVEN_STATE_12 */ { 1500, 750, BATTERY_SEVEN_STATE_13 },
    /* BATTERY_SEVEN_STATE_13 */ { 0, 500, BATTERY_SEVEN_STATE_14 },
    /* BATTERY_SEVEN_STATE_14 */ { 1500, 750, BATTERY_SEVEN_STATE_15 },
    /* BATTERY_SEVEN_STATE_15 */ { 0, 500, BATTERY_SEVEN_STATE_16 },
    /* BATTERY_SEVEN_STATE_16 */ { 1500, 750, BATTERY_SEVEN_STATE_17 },
    /* BATTERY_SEVEN_STATE_17 */ { 0, 500, BATTERY_SEVEN_STATE_18 },
    /* BATTERY_SEVEN_STATE_18 */ { 1500, 750, BATTERY_SEVEN_STATE_19 },
    /* BATTERY_SEVEN_STATE_19 */ { 0, 500, INITIAL },
    /* BATTERY_EIGHT_STATE_0 */ { 1000, 250, BATTERY_EIGHT_STATE_1 },
    /* BATTERY_EIGHT_STATE_1 */ { 0, 500, BATTERY_EIGHT_STATE_2 },
    /* BATTERY_EIGHT_STATE_2 */ { 1500, 750, BATTERY_EIGHT_STATE_3 },
    /* BATTERY_EIGHT_STATE_3 */ { 0, 500, BATTERY_EIGHT_STATE_4 },
    /* BATTERY_EIGHT_STATE_4 */ { 1500, 750, BATTERY_EIGHT_STATE_5 },
    /* BATTERY_EIGHT_STATE_5 */ { 0, 500, BATTERY_EIGHT_STATE_6 },
    /* BATTERY_EIGHT_STATE_6 */ { 1500, 750, BATTERY_EIGHT_STATE_7 },
    /* BATTERY_EIGHT_STATE_7 */ { 0, 500, BATTERY_EIGHT_STATE_8 },
    /* BATTERY_EIGHT_STATE_8 */ { 1500, 750, BATTERY_EIGHT_STATE_9 },
    /* BATTERY_EIGHT_STATE_9 */ { 0, 500, BATTERY_EIGHT_STATE_10 },
    /* BATTERY_EIGHT_STATE_10 */ { 1500, 750, BATTERY_EIGHT_STATE_11 },
    /* BATTERY_EIGHT_STATE_11 */ { 0, 500, BATTERY_EIGHT_STATE_12 },
    /* BATTERY_EIGHT_STATE_12 */ { 1500, 750, BATTERY_EIGHT_STATE_13 },
    /* BATTERY_EIGHT_STATE_13 */ { 0, 500, BATTERY_EIGHT_STATE_14 },
    /* BATTERY_EIGHT_STATE_14 */ { 1500, 750, BATTERY_EIGHT_STATE_15 },
    /* BATTERY_EIGHT_STATE_15 */ { 0, 500, BATTERY_EIGHT_STATE_16 },
    /* BATTERY_EIGHT_STATE_16 */ { 1500, 750, BATTERY_EIGHT_STATE_17 },
    /* BATTERY_EIGHT_STATE_17 */ { 0, 500, INITIAL },
    /* BATTERY_NINE_STATE_0 */ { 1000, 250, BATTERY_NINE_STATE_1 },
    /* BATTERY_NINE_STATE_1 */ { 0, 500, BATTERY_NINE_STATE_2 },
    /* BATTERY_NINE_STATE_2 */ { 1500, 750, BATTERY_NINE_STATE_3 },
    /* BATTERY_NINE_STATE_3 */ { 0, 500, BATTERY_NINE_STATE_4 },
    /* BATTERY_NINE_STATE_4 */ { 1500, 750, BATTERY_NINE_STATE_5 },
    /* BATTERY_NINE_STATE_5 */ { 0, 500, BATTERY_NINE_STATE_6 },
    /* BATTERY_NINE_STATE_6 */ { 1500, 750, BATTERY_NINE_STATE_7 },
    /* BATTERY_NINE_STATE_7 */ { 0, 500, BATTERY_NINE_STATE_8 },
    /* BATTERY_NINE_STATE_8 */ { 1500, 750, BATTERY_NINE_STATE_9 },
    /* BATTERY_NINE_STATE_9 */ { 0, 500, BATTERY_NINE_STATE_10 },
    /* BATTERY_NINE_STATE_10 */ { 1500, 750, BATTERY_NINE_STATE_11 },
    /* BATTERY_NINE_STATE_11 */ { 0, 500, BATTERY_NINE_STATE_12 },
    /* BATTERY_NINE_STATE_12 */ { 1500, 750, BATTERY_NINE_STATE_13 },
    /* BATTERY_NINE_STATE_13 */ { 0, 500, BATTERY_NINE_STATE_14 },
    /* BATTERY_NINE_STATE_14 */ { 1500, 750, BATTERY_NINE_STATE_15 },
    /* BATTERY_NINE_STATE_15 */ { 0, 500, BATTERY_NINE_STATE_16 },
    /* BATTERY_NINE_STATE_16 */ { 1500, 750, BATTERY_NINE_STATE_17 },
    /* BATTERY_NINE_STATE_17 */ { 0, 500, BATTERY_NINE_STATE_18 },
    /* BATTERY_NINE_STATE_18 */ { 1500, 750, BATTERY_NINE_STATE_19 },
    /* BATTERY_NINE_STATE_19 */ { 0, 500, INITIAL },
};

/* ======= END AUTOGENERATED ======= */

enum BuzzerControlState {
    WAITING_FOR_BATTERY,
    SENDING_BATTERY,
    SENDING_FSM_STATE
};

class BuzzerController {
public:
    explicit BuzzerController(pointers *);

    void tickBuzzer();
    void setBuzzerState(BuzzerState state);

private:
    BuzzerControlState control_state;

    BuzzerState curr_state;
    unsigned long time_since_state_start;

    pointers* rocket_state;
    FSM_State last_rocket_fsm_state;
};


#endif