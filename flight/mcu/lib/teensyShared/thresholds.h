
#ifndef THRESHOLDS_H
#define THRESHOLDS_H

// TODO set values for the thresholds
static const float coast_time_thresh = 300;
static const float coast_thresh = 0;
static const float apogee_thresh = 0;
static const float main_thresh_min =
    -2;  // still not determined, check open rocket
static const float main_thresh_max = -.1;   // still not determined
static const float drogue_thresh_max = -2;  // still not determined
static const float drogue_thresh_min = -2;  // still not determined
static const float landed_thresh = -.1;
static const float launch_az_thresh =
    1;  // Minimum vert acc. for launch detection
static const float launch_time_thresh = 300;  // Minimum time to confirm launch
static const float apogee_time_thresh = 3000;
static const float drogue_time_thresh = 2500;
static const float main_time_thresh = 2500;
static const float landing_time_thresh = 300;

static bool launch_init = false;
static bool coast_init = false;

#endif
