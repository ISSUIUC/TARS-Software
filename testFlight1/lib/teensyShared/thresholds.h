#ifndef THRESHOLDS_H
#define THRESHOLDS_H
 
//TODO set values for the thresholds
float coast_time_thresh = 300;
float coast_thresh = 0;
float apogee_thresh = 0;
float drogue_thresh_min = 1; //still not determined
float drogue_thresh_max = 2; //still not determined
float main_thresh = 2; //still not determined
float landed_thresh = -.1;
float coast_timer;
float descent_timer;
float burn_timer; //Measuring how long the burn happens
float launch_az_thresh = 1; //Minimum vert acc. for launch detection
float launch_time_thresh = 300; //Minimum time to confirm launch
float apogee_time_thresh = 300;
float drogue_time_thresh = 300;
float main_time_thresh = 300;
float landing_time_thresh = 300;
bool launch_init = false;
bool coast_init = false;
 
#endif

