/**
 * @file        TimerFSM.cpp
 * @authors     Anshuk Chigullapalli
 * 		        Ayberk Yaraneri
 * 		        Colin Kinsey
 *              Gautam Dayal
 *              Jusjeev Singh Bhurjee
 *              Rithvik Bhogavilli
 *              Nicholas Phillips
 *              Aidan Costello
 *              Aaditya Voruganti
 *              Jessica Myers
 *
 * @brief      The implementation of the finite state machine class that governs
 * state transitions.
 *
 * The TimerFSM is a collection of barebones logic that only uses vertical acceleration to detect BOOST and BURNOUT.
 * After these states, the transitions are entirely dictated by predetermined time intervals that are calculated using
 * OpenRocket and/or SILSIM simulations. This FSM is useful as a sort of control variable to compare against the other
 * ones, and also ensures that the GNC flaps do not retract at the wrong time.
 *
 * This is a highly critical software module and should be tested throughly in
 * simulation and on hardware targets.
 *
 */

#pragma once

#include "mcu_main/finite-state-machines/RocketFSMBase.h"

class TimerFSM : public RocketFSMBase {
   public:
    TimerFSM() = default;

    /**
     * @brief TimerFSM tick function
     *
     * Uses a combination of linear acceleration and timers to govern FSM state
     * changes for each timestep of the rocket's flight.
     */
    void tickFSM() override;

   private:
    systime_t launch_time_ = 0;
    sysinterval_t burn_timer_ = 0;
    systime_t burnout_time_ = 0;
    sysinterval_t coast_timer_ = 0;

    systime_t apogee_time_ = 0;
    sysinterval_t apogee_timer_ = 0;

    systime_t drogue_time_ = 0;
    sysinterval_t drogue_timer_ = 0;

    systime_t main_time_ = 0;
    sysinterval_t main_timer_ = 0;

    systime_t landing_time_ = 0;
    sysinterval_t landing_timer = 0;
};
