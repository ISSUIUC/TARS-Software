/** @file thresholds.h
 *
 * @brief Contains thresholds for time and acceleration that
 * are used to determine changes in FSM state.
 */

#ifndef THRESHOLDS_H
#define THRESHOLDS_H

// Transition from boost to coast if low accleration detected for 300 ms
static const float coast_time_thresh = 300;

//transition from coast to apogee if low velocity detected for 300ms
static const float apogee_time_thresh = 300;

// 2 gs for launch detect
static const float launch_linear_acceleration_thresh = 2;

// Required time of detected acceleration to switch to boost
static const float launch_time_thresh = 20;

// If we observe less than 0.5 g of accel, transition to coast
static const float coast_thresh = 0.5;

// Switch to apogee state after 40 seconds
static const float coast_to_apogee_time_thresh = 5000;

// For preventing GNC flaps from actuating even when in coast state
static const float coast_ac_delay_thresh = 15000;

// Rocket will leave boost state when burn_timer has exceeded this limit
static const int burn_time_thresh_ms = 5000;

// how small the difference in altimeter data can be before it detects apogee (~ 0, a very small number of some kind)
// TODO: tune
static const float apogee_altimeter_threshold = -1;

// how large the change in acceleration has to be to detect landing:
static const float landing_imu_threshold = 0.1;

static const float landing_altimeter_threshold = .1;

// some number greater than 0 to represent a drastic change in acceleration being detected to put us into drogue:
static const float drogue_acceleration_change_threshold_imu = 0.1;

static const float drogue_acceleration_change_threshold_altimeter = 0.1;

// some number greater than 0 to represent a drastic change in acceleration being detected to put us into main:
static const float main_acceleration_change_threshold_imu = 0.1;

static const float main_acceleration_change_threshold_altimeter = 0.1;

// how long before we force droge deploy
static const float drogue_deploy_time_since_apogee_threshold = 10000;

//how long before we force main deploy
static const float main_deploy_time_since_drogue_threshold = 10000;

//how long we wait to let the buffers cycle
static const float refresh_timer = 500;

#endif
