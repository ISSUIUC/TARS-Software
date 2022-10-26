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
static const float launch_time_thresh = 300;

// If we observe less than 0.5 g of accel, transition to coast
static const float coast_thresh = 0.5;

// Switch to apogee state after 26.3 seconds
static const float coast_to_apogee_time_thresh = 26300;

// For preventing AC from actuating even when in coast state
static const float coast_ac_delay_thresh = 1000;

// Rocket won't leave boost state until burn_timer has exceeded this limit
static const int burn_time_thresh_ms = 5000;

#endif
