/** @file thresholds.h
 *
 * @brief Contains thresholds for time and acceleration that
 * are used to determine changes in FSM state.
 */

#ifndef THRESHOLDS_H
#define THRESHOLDS_H

// Transition from boost to coast if low accleration detected for 300 ms
static const float coast_time_thresh = 300;

// 2 gs for launch detect
static const float launch_linear_acceleration_thresh = 2;

// Required time of detected acceleration to switch to boost
static const float launch_time_thresh = 20;

// If we observe less than 0.5 g of accel, transition to coast
static const float coast_thresh = 0.5;

// Switch to apogee state after 40 seconds
static const float coast_to_apogee_time_thresh = 5000;

// For preventing AC from actuating even when in coast state
static const float coast_ac_delay_thresh = 15000;

// Rocket won't leave boost state until burn_timer has exceeded this limit
static const int burn_time_thresh_ms = 5000;

// how small the difference in altimeter data can be before it detects apogee (~ 0, a very small number of some kind)
// TODO: tune
static const float apogee_altimeter_threshold = -1;

// see if the change in altitude is close to 0 (or some small number) to detect landing:
// TODO: tune
static const float landing_imu_threshold = 0.1;

static const float landing_altimeter_threshold = .1;

// some number greater than 0 to represent a drastic change in acceleration being detected:
// TODO: tune
static const float drogue_acceleration_change_threshold_imu = 0.1;

static const float drogue_acceleration_change_threshold_altimeter = 0.1;

// some number greater than 0 to represent a drastic change in acceleration being detected:
// TODO: tune
static const float main_acceleration_change_threshold_imu = 0.1;

static const float main_acceleration_change_threshold_altimeter = 0.1;

// this comes from SILSIM (in milliseconds):
// TODO: tune
static const float drogue_deploy_time_since_apogee_threshold = 10000;

static const float main_deploy_time_since_drogue_threshold = 10000;

static const float refresh_timer = 500;

#endif
