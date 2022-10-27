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
 * 
 *              Jessica Myers
 *              Aidan Costello
 *              Aaditya Voruganti
 *
 * @brief      The implementation of the finite state machine class that governs
 * state transitions.
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

#include "KalmanFSM.h"

#include "dataLog.h"
#include "pins.h"
#include "thresholds.h"

#include <cmath>

/**
 * @brief Constructor for KalmanFSM class
 * @param pointers
 *
 * Taking the pointer struct as an input, we define member variables that point
 * to different data members of pointer struct. This improves readability while
 * also allowing the rocketFSM class to modify values in the global pointer
 * struct.
 */
KalmanFSM::KalmanFSM(pointers *ptr) {
    pointer_struct = ptr;
    // Get the linear accelration from the High-G IMU
    gnc_state_ptr_ = &pointer_struct->sensorDataPointer->state_data;

    altitude_history_ptr_ = &pointer_struct->dataloggerTHDVarsPointer.gnc_altitude_history_6;
    IMU_acceleration_history_ptr_ = &pointer_struct->dataloggerTHDVarsPointer.gnc_IMU_acceleration_history_6;
}

/**
 * @brief KalmanFSM tick function
 *
 * Uses a combination of linear acceleration and timers to govern FSM state
 * changes for each timestep of the rocket's flight.
 */
void KalmanFSM::tickFSM() {
    // Lock mutexes for data used in switch
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    //Serial.println(state_map[(int)rocket_state_]);

    // Links to abort for other states
    if (pointer_struct->abort) {
        rocket_state_ = FSM_State::STATE_ABORT;
    }

    switch (rocket_state_) {
        case FSM_State::STATE_ABORT:
            // If true, always stay in abort
            break;

        case FSM_State::STATE_INIT:
            // Go to state idle regardless of gps lock
            rocket_state_ = FSM_State::STATE_IDLE;

            break;

        case FSM_State::STATE_IDLE:
            // If high acceleration is observed in z direction...
            if (gnc_state_ptr_->state_ax > launch_linear_acceleration_thresh) {
                launch_time_ = chVTGetSystemTime();
                rocket_state_ = FSM_State::STATE_LAUNCH_DETECT;
            }

            break;

        case FSM_State::STATE_LAUNCH_DETECT:
            // If the acceleration was too brief, go back to IDLE
            if (gnc_state_ptr_->state_ax < launch_linear_acceleration_thresh) {
                rocket_state_ = FSM_State::STATE_IDLE;
                break;
            }

            // Measure the length of the burn time (for hysteresis)
            burn_timer_ = chVTGetSystemTime() - launch_time_;

            // If the acceleration lasts long enough, boost is detected
            if (TIME_I2MS(burn_timer_) > launch_time_thresh) {
                rocket_state_ = FSM_State::STATE_BOOST;
            }

            break;

        case FSM_State::STATE_BOOST:
            burn_timer_ = chVTGetSystemTime() - launch_time_;
            // If low acceleration in the Z direction...
            if (gnc_state_ptr_->state_ax < coast_thresh) {
                // Serial.println("Acceleration below thresh");
                burnout_time_ = chVTGetSystemTime();
                rocket_state_ = FSM_State::STATE_BURNOUT_DETECT;
                break;
            }
            // Keeping rocket in FSM_State::STATE_BOOST if time below a certain
            // threshold
            if (TIME_I2MS(burn_timer_) < burn_time_thresh_ms) {
                rocket_state_ = FSM_State::STATE_BOOST;
            }
            // Forcing rocket to go to FSM_State::STATE_COAST if threshold
            // crossed
            else {
                rocket_state_ = FSM_State::STATE_COAST_PREGNC;
                // Setting burnout time because we don't otherwise
                burnout_time_ = chVTGetSystemTime();
            }

            break;

        case FSM_State::STATE_BURNOUT_DETECT:
            // If the 0 acceleration was too brief, go back to BOOST
            if (gnc_state_ptr_->state_ax > coast_thresh) {
                rocket_state_ = FSM_State::STATE_BOOST;
                break;
            }

            // Measure the length of the coast time (for hysteresis)
            coast_timer_ = chVTGetSystemTime() - burnout_time_;

            // If the low acceleration lasts long enough, coast is detected
            if (TIME_I2MS(coast_timer_) > coast_time_thresh) {
                rocket_state_ = FSM_State::STATE_COAST_PREGNC;
            }

            break;

        case FSM_State::STATE_COAST_PREGNC:
            coast_timer_ = chVTGetSystemTime() - burnout_time_;
            if (TIME_I2MS(coast_timer_) > coast_ac_delay_thresh) {
                rocket_state_ = FSM_State::STATE_COAST_GNC;
            }

            break;

        case FSM_State::STATE_COAST_GNC:
            coast_timer_ = chVTGetSystemTime() - burnout_time_;

            if (fabs(gnc_state_ptr_->state_vx)*0.02 < apogee_altimeter_threshold) {
                    rocket_state_ = FSM_State::STATE_APOGEE_DETECT;
                    apogee_time_ = chVTGetSystemTime();
                    break;
            }
            // float changevelo = (fabs((*altitude_history_ptr_).getCurrentAverage() - (*altitude_history_ptr_).getPastAverage()))/0.12;
            // coast_to_apogee_time_thresh = changevelo/fabs((*altitude_history_ptr_).getCurrentSecondDerivativeAverage());

            if (TIME_I2MS(coast_timer_) < coast_to_apogee_time_thresh) {
                rocket_state_ = FSM_State::STATE_COAST_GNC;
            }
            else {
                rocket_state_ = FSM_State::STATE_APOGEE;
                apogee_time_ = chVTGetSystemTime();
            }

            break;

        case FSM_State::STATE_APOGEE_DETECT:
            // If the 0 velocity was too brief, go back to coast
            if (fabs(gnc_state_ptr_->state_vx)*0.02 > apogee_altimeter_threshold) {
                rocket_state_ = FSM_State::STATE_COAST_GNC;
                break;
            }

            // Measure the length of the apogee time (for hysteresis)
            apogee_timer_ = chVTGetSystemTime() - apogee_time_;

            // If the low velocity lasts long enough, apogee is detected
            if (TIME_I2MS(apogee_timer_) > apogee_time_thresh) {
                rocket_state_ = FSM_State::STATE_APOGEE;
            }

            break;


        case FSM_State::STATE_APOGEE:
            if (fabs((*IMU_acceleration_history_ptr_).getCurrentAverage() - (*IMU_acceleration_history_ptr_).getPastAverage()) > drogue_acceleration_change_threshold_imu) {
                    rocket_state_= FSM_State::STATE_DROGUE_DETECT;
                    break;
            }
            //potentially add back state to put us back into coast

            
            if (TIME_I2MS(apogee_timer_) < drogue_deploy_time_since_apogee_threshold) {
                rocket_state_ = FSM_State::STATE_APOGEE;
            }
            else {
                rocket_state_ = FSM_State::STATE_DROGUE;
                drogue_time_ = chVTGetSystemTime();
            }
            break;

        case FSM_State::STATE_DROGUE_DETECT:
            if (fabs((*altitude_history_ptr_).getCurrentSecondDerivativeAverage() - (*altitude_history_ptr_).getPastSecondDerivativeAverage()) > drogue_acceleration_change_threshold_altimeter) {
                    rocket_state_ = FSM_State::STATE_DROGUE;
                    drogue_time_ = chVTGetSystemTime();
                    break;
            }

            else{
                rocket_state_ = FSM_State::STATE_APOGEE;
            }
            break;

        case FSM_State::STATE_DROGUE:
            drogue_timer_ = chVTGetSystemTime() - drogue_time_;
            if(TIME_I2MS(drogue_timer_) > refresh_timer){
                if (fabs((*IMU_acceleration_history_ptr_).getCurrentAverage() - (*IMU_acceleration_history_ptr_).getPastAverage()) > main_acceleration_change_threshold_imu) {
                        rocket_state_= FSM_State::STATE_MAIN_DETECT;
                        break;
                }
            }
            

            if (TIME_I2MS(drogue_timer_) < main_deploy_time_since_drogue_threshold) {
                rocket_state_ = FSM_State::STATE_DROGUE;
                
            }
            else {
                rocket_state_ = FSM_State::STATE_MAIN;
                main_time_ = chVTGetSystemTime();
            }
            break;

        case FSM_State::STATE_MAIN_DETECT:
            if (fabs((*altitude_history_ptr_).getCurrentSecondDerivativeAverage() - (*altitude_history_ptr_).getPastSecondDerivativeAverage()) > main_acceleration_change_threshold_altimeter) {
                    rocket_state_ = FSM_State::STATE_MAIN;
                    main_time_ = chVTGetSystemTime();
                    break;
            }
            else{
                rocket_state_ = FSM_State::STATE_DROGUE;
            }
            break;

        case FSM_State::STATE_MAIN:
            main_timer_ = chVTGetSystemTime() - main_time_;

            // if(TIME_I2MS(main_timer_) > refresh_timer){
            if (fabs((*altitude_history_ptr_).getCurrentAverage() - (*altitude_history_ptr_).getPastAverage()) < landing_altimeter_threshold) {
                rocket_state_ = FSM_State::STATE_LANDED_DETECT;
                landing_time_ = chVTGetSystemTime();
                break;
            }
            // }
            
            if (TIME_I2MS(main_timer_) < main_deploy_time_since_drogue_threshold) {
                rocket_state_ = FSM_State::STATE_MAIN;
                
            }
            else {
                rocket_state_ = FSM_State::STATE_LANDED;
                landing_time_ = chVTGetSystemTime();
            }
            break;

        case FSM_State::STATE_LANDED_DETECT:
            // If the 0 velocity was too brief, go back to main
            if (fabs((*altitude_history_ptr_).getCurrentAverage() - (*altitude_history_ptr_).getPastAverage()) > landing_altimeter_threshold) {
                rocket_state_ = FSM_State::STATE_MAIN;
                break;
            }

            // Measure the length of the landed time (for hysteresis)
            landing_timer = chVTGetSystemTime() - landing_time_;

            // If the low velocity lasts long enough, landing is detected
            if (TIME_I2MS(landing_timer) > landing_time_thresh) {
                rocket_state_ = FSM_State::STATE_LANDED;
            }

            break;

        case FSM_State::STATE_LANDED:
            break;


        default:
            break;
    }

    // Unlock mutexes used during the switch statement
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);
}
