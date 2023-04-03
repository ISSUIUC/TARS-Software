/**
 * @file        KalmanFSM.cpp
 * @authors     Anshuk Chigullapalli
 * 		        Ayberk Yaraneri
 * 		        Colin Kinsey
 *              Gautam Dayal
 *              Jusjeev Singh Bhurjee
 *              Rithvik Bhogavilli
 *              Nicholas Phillips
 *
 *              Jessica Myers
 *              Aidan Costello
 *              Aaditya Voruganti
 *
 *              Magilan Sendhil
 *
 * @brief      The implementation of the finite state machine class that governs state transitions.
 *
 * The KalmanFSM class encapsulates the finite state machine that dictates which
 * state the rocket is in throughout the mission. The class implements the logic
 * necessary to reliably transition between states along with hysteresis to
 * avoid premature state transitions.
 *
 * This is a highly critical software module and should be tested throughly in
 * simulation and on hardware targets.:
 *
 */

#pragma once

#include "mcu_main/finite-state-machines/RocketFSMBase.h"

class KalmanFSM : public RocketFSMBase {
   public:
    KalmanFSM() = default;

    /**
    * @brief KalmanFSM tick function
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

    double getAltitudeAverage(size_t start, size_t len);
    double getSecondDerivativeAltitudeAverage(size_t start, size_t len);
    double getAccelerationAverage(size_t start, size_t len);
};
