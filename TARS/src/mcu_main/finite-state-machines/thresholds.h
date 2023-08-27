/** @file thresholds.h
 *
 * @brief Contains thresholds for time and acceleration that
 * are used to determine changes in FSM state.
 */

#pragma once
#include <cmath>
// Transition from boost to coast if low acceleration detected for 300 ms
static constexpr float coast_time_thresh = 300;

// transition from coast to apogee if low velocity detected for 100ms
static constexpr float apogee_time_thresh = 100;

static constexpr float landing_time_thresh = 1000;

// 2 gs for launch detect
static constexpr float launch_linear_acceleration_thresh = 3;

// Required time of detected acceleration to switch to boost
static constexpr float launch_time_thresh = 250;

// If we observe less than 0.5 g of accel, transition to coast
static constexpr float coast_thresh = 0.2;

// Switch to apogee state after 40 seconds
static constexpr float coast_to_apogee_time_thresh = 26300;

// For preventing GNC flaps from actuating even when in coast state
static constexpr float coast_ac_delay_thresh = 8121.4;

// Rocket will leave boost state when burn_timer has exceeded this limit
static constexpr int burn_time_thresh_ms = 5200;

// how small the difference in altimeter data can be before it detects apogee
static constexpr float apogee_altimeter_threshold = 0.5;

// some number greater than 0 to represent a drastic change in acceleration
// being detected to put us into drogue:
static constexpr float drogue_acceleration_change_threshold_imu = 0.15;

static constexpr float drogue_acceleration_change_threshold_altimeter = 10.01;

// some number greater than 0 to represent a drastic change in acceleration
// being detected to put us into main:
static constexpr float main_acceleration_change_threshold_imu = 1.53;

static constexpr float main_acceleration_change_threshold_altimeter = 25.02;

// how long before we force drogue deploy
static constexpr float drogue_deploy_time_since_apogee_threshold = 10000;

// how long before we force main deploy
static constexpr float main_deploy_time_since_drogue_threshold = 105000;

// how large the change in altitude has to be to detect landing:
static constexpr float landing_altimeter_threshold = .5;

// how long we wait to let the buffers cycle
static constexpr float refresh_timer = 500;

// start angle
static constexpr float ang_start = 0;

// launch site error
static constexpr float alt_error = 20;

// idle acceleration error
static constexpr float acc_error = 0.75;

// idle angle error
static constexpr float ang_error = 45 / 180.0 * M_PI;

// idle velocity error
static constexpr float vel_error = 5;

// boost acceleration tolerance
static constexpr float boost_to_coast_acceleration = 1;

// lower bound for allowable acceleration for boost

static constexpr float drogue_to_main_acceleration = -4;
// lower bound for allowable accerleation for boost
static constexpr float boost_acc_thresh = 1;

// bounds for allowable angle for boost (radians)
static constexpr float boost_ang_thresh = 85 / 180.0 * M_PI;

// coast gnc angle threshold
static constexpr float coast_gnc_thresh = 30 / 180.0 * M_PI;

// main angle bottom threshold
static constexpr float main_ang_thresh_bottom = 95 / 180.0 * M_PI;

// main angle bottom threshold
static constexpr float main_ang_thresh_top = -95 / 180.0 * M_PI;

// drogue angle bottom threshold
static constexpr float drogue_ang_thresh_bottom = 95 / 180.0 * M_PI;

// drogue angle bottom threshold
static constexpr float drogue_ang_thresh_top = -95 / 180.0 * M_PI;

// drogue acceleration bottom threshold
static constexpr float drogue_acc_bottom = -4;

// drogue acceleration bottom threshold
static constexpr float drogue_acc_top = 1;

// main acceleration top threshold
static constexpr float main_acc_top = -4;

// acceleration to go from apogee to separation
static constexpr float apogee_to_separation_acceleration = 4;

// acceleration to go from separation to drogue
static constexpr float separation_to_drogue_acceleration = -0.5;

// lower bound for allowable acceleration for separation
static constexpr float separation_acc_thresh = 4;
