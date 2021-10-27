
#ifndef THRESHOLDS_H
#define THRESHOLDS_H

// TODO set values for the thresholds
static const float coast_time_thresh = 50;
//2 gs for launch detect
static const float launch_az_thresh = 2;
//required time of detected acceleration to switch to boost
static const float launch_time_thresh = 50;
//TODO change this
static const float coast_thresh = 0.5;
//switch to apogee state
static const float coast_to_apogee_time_threash = 20000;

static bool launch_init = false;
static bool coast_init = false;

#endif
