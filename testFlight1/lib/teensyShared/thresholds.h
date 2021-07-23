#ifndef THRESHOLDS_H
#define THRESHOLDS_H
 
//TODO set values for the thresholds
float const coast_time_thresh = 300;
float const coast_thresh = 0;
float const apogee_thresh = 0;
float const main_thresh_min = -2; //still not determined, check open rocket
float const main_thresh_max = -.1; //still not determined
float const drogue_thresh_max = -2; //still not determined
float const drogue_thresh_min = -2; //still not determined
float const landed_thresh = -.1;
float const launch_az_thresh = 1; //Minimum vert acc. for launch detection
float const launch_time_thresh = 300; //Minimum time to confirm launch
float const apogee_time_thresh = 3000;
float const drogue_time_thresh = 2500;
float const main_time_thresh = 2500;
float const landing_time_thresh = 300;
bool launch_init = false;
bool coast_init = false;
 
#endif

