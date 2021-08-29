#ifndef ACSHARED_H
#define ACSHARED_H

//data structure for FSM States
struct fsm_struct{
    int rocket_state; //This is the rocket state. 0 is IDLE, 1 is LAUNCHED, 2 is COAST, 3 is APOGEE
    float launch_time; //First time acceleration above threshold is detected
    float burnout_time;
    float apogee_time;
};

#endif