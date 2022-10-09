/**
 * @file BuzzerControl.hpp
 *
 * Public interface of the BuzzerController
 */

#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include "dataLog.h"
#include "acShared.h"

// the following tables should be generated by beep_fsm_gen.py
// TODO this doesn't need a full fsm, we can get away with a struct of a sequence and a "should_loop" parameter
// TODO it would make the code a lot simpler and more comprehensible
/* ======= AUTOGENERATED 1 ======= */

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

/* ======= END AUTOGENERATED 1 ======= */

/**
 * @brief A table which stores the necessary information for running every beep sequence
 *
 * You can index into this table with a BuzzerState casted into an int. This will yield an array which has three elements:
 * The first item is the pitch to play at this beep (or 0 if it's to be silent).
 * The second item is how long to hold this beep (or silence) for in milliseconds.
 * The third item is the BuzzerState to transition to after this state finishes.
 *
 * By setting the third item to the BuzzerState for the first item in this beep sequence, you can make a looping beep sequence.
 */

/* ======= AUTOGENERATED 2 ======= */
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

/* ======= END AUTOGENERATED 2 ======= */

/**
 * @brief The variants specify which control state the Buzzer is in, which is used to allow the Buzzer to fully transmit
 * its battery voltage before transitioning to the mode where it beeps out its Rocket FSM state.
 */
enum BuzzerControlState {
    /**
     * @brief The initial state, it signals to the BuzzerController that it should wait until some voltage data is read in.
     */
    WAITING_FOR_BATTERY,

    /**
     * @brief The 'transmitting battery voltage state', it signals to the BuzzerController that some battery beep sequence is playing and
     * that it should wait until it's over before starting to send Rocket FSM states.
     */
    SENDING_BATTERY,

    /**
     * @brief The core state, it signals to the BuzzerController that it should check for Rocket FSM state changes and switch
     * beep sequences based on that.
     */
    SENDING_FSM_STATE
};

/**
 * @brief Holds the state of the Buzzer and uses data from the rocket FSM and the battery to control beeps.
 *
 * The BuzzerController class holds all the necessary data that it needs to control the buzzer. In an Arduino, the
 * tone() and noTone() functions start and stop generating a square wave on some particular pin. The most straightforward
 * way to generate a beep using these functions would be to call tone(), sleep for some amount of time, then call noTone().
 * However, this doesn't play well with how ChibiOS does thread switching, nor does it allow changing which beep sequence
 * you're in midway through a beep, since you're sleeping for the duration of the beep.
 *
 * Instead, what we do instead is generate an FSM specifically for the buzzer. This FSM holds all the data needed for
 * tracking what pitch a beep should have, how long that beep (or silence) should hold for, and what FSM state to go to
 * next. This is why the huge autogenerated tables BuzzerStates and BuzzerState exist.
 *
 */
class BuzzerController {
public:
    /**
     * @brief Constructs a BuzzerController, taking all the data it needs from a pointers*.
     */
    explicit BuzzerController(pointers *);

    /**
     * @brief The core of the BuzzerController class, it updates what beep sequence its on based on its control state, and
     * updates its position in that sequence based on the autogenerated tables.
     */
    void tickBuzzer();

    /**
     * @brief Sets the Buzzer State to the argument.
     *
     * @param state A Buzzer FSM state to set the BuzzerController to. Typically should be either one that ends in '_0'
     * or BuzzerState::Initial.
     */
    void setBuzzerState(BuzzerState state);

private:
    /**
     * @brief Given a battery voltage, compute what BuzzerState we should go to (some BATTERY_ sequence), or
     * BuzzerState::Initial if we shouldn't be on one.
     *
     * @param battery_voltage The battery's current voltage, which should be less than or equal to 3.3 V.
     * @return A BuzzerState that corresponds to the start of a BATTERY_ sequence, or BuzzerState::Initial in case of error.
     */
    static BuzzerState getNewStateFromBattery(float battery_voltage);

    /**
     * @brief Given a rocket FSM state, return what BuzzerState we should go to, or BuzzerState::Initial if we shouldn't change sequences.
     *
     * @param rocket_state The rocket's current FSM state.
     * @return A BuzzerState that corresponds to what sequence we should transition to, or BuzzerState::Initial if rocket_state was
     * some signalling state we should ignore.
     */
    static BuzzerState getNewStateFromRocket(FSM_State rocket_state);

    /**
     * @brief The control state, which controls whether to be signalling battery voltage or FSM state.
     */
    BuzzerControlState control_state;

    /**
     * @brief The current Buzzer FSM state, used to index into the BuzzerStates table and find pitch, duration, and
     * next state.
     */
    BuzzerState curr_state;
    /**
     * @brief The timestamp returned from millis() of when we entered this state. Once the time elapsed becomes equal to
     * the duration found by indexing into BuzzerStates, we transition to the next state (also found by indexing into BuzzerStates).
     */
    unsigned long state_start_time;

    pointers* rocket_state;
    /**
     * @brief Stores the last known state of the RocketFSM. When this changes, we know to enter a new beep sequence.
     */
    FSM_State last_rocket_fsm_state;
};


#endif