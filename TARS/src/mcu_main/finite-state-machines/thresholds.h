/** @file thresholds.h
 *
 * @brief Contains thresholds for time and acceleration that
 * are used to determine changes in FSM state.
 */

#pragma once

// Transition from boost to coast if low accleration detected for 300 ms
static const float coast_time_thresh = 300;

// transition from coast to apogee if low velocity detected for 100ms
static const float apogee_time_thresh = 100;

static const float landing_time_thresh = 1000;

// 2 gs for launch detect
static const float launch_linear_acceleration_thresh = 3;

// Required time of detected acceleration to switch to boost
static const float launch_time_thresh = 250;

// If we observe less than 0.5 g of accel, transition to coast
static const float coast_thresh = 0.5;

// Switch to apogee state after 40 seconds
static const float coast_to_apogee_time_thresh = 26300;

// For preventing GNC flaps from actuating even when in coast state
static const float coast_ac_delay_thresh = 1000;

// Rocket will leave boost state when burn_timer has exceeded this limit
static const int burn_time_thresh_ms = 5200;

// how small the difference in altimeter data can be before it detects apogee
static const float apogee_altimeter_threshold = 0.5;

// some number greater than 0 to represent a drastic change in acceleration
// being detected to put us into drogue:
static const float drogue_acceleration_change_threshold_imu = 0.15;

static const float drogue_acceleration_change_threshold_altimeter = 10.01;

// some number greater than 0 to represent a drastic change in acceleration
// being detected to put us into main:
static const float main_acceleration_change_threshold_imu = 1.53;

static const float main_acceleration_change_threshold_altimeter = 25.02;

// how long before we force droge deploy
static const float drogue_deploy_time_since_apogee_threshold = 10000;

// how long before we force main deploy
static const float main_deploy_time_since_drogue_threshold = 105000;

// how large the change in altitude has to be to detect landing:
static const float landing_altimeter_threshold = .5;

// how long we wait to let the buffers cycle
static const float refresh_timer = 500;

// launch site altitude as used in modular fsm
static const float launch_site_altitude = 0;

// start angle
static const float ang_start = 0;

// launch site error
static const float alt_error = 5;

// idle acceleration error
static const float acc_error = 0.5;

// idle angle error
static const float ang_error = 45;

// idle velocity error
static const float vel_error = 1;

//boost acceleration tolerance
static const float boost_to_coast_acceleration = 1;

//lower bound for allowable accerleation for boost
static const float boost_acc_thresh = 1;

//bounds for allowable angle for boost (degrees)
static const float boost_ang_thresh = 85;
