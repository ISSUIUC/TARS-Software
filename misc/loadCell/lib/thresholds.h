#ifndef THRESHOLDS_H
#define THRESHOLDS_H

//TODO set values for the thresholds
float coast_time_thresh = 500;
float coast_thresh = 100;
float coast_timer;
float descent_timer;
float burn_timer; //Measuring how long the burn happens
float launch_linear_acceleration_thresh = 3500; //Minimum vert acc. for launch detection
float launch_time_thresh = 300; //Minimum time to confirm launch
float apogee_time_thresh;
bool launch_init = false;
bool coast_init = false;

#endif
