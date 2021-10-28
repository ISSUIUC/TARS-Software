
#ifndef THRESHOLDS_H
#define THRESHOLDS_H

// transition from boost to coast if low accleration detected for 300 ms
static const float coast_time_thresh = 300;
// 2 gs for launch detect
static const float launch_linear_acceleration_thresh = 2;
// required time of detected acceleration to switch to boost
static const float launch_time_thresh = 300;
// if we observe less than 0.5 g of accel, transition to coast
static const float coast_thresh = 0.5;
// switch to apogee state after 20 seconds
static const float coast_to_apogee_time_threash = 20000;

#endif
