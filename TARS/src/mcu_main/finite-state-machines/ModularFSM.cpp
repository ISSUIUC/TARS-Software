#include "mcu_main/finite-state-machines/ModularFSM.h"

#include "mcu_main/finite-state-machines/thresholds.h"
#include "mcu_main/sensors/sensors.h"
#include "mcu_main/dataLog.h"

double ModularFSM::getAltitudeAverage(size_t start, size_t len) {
    return ModularFSM::getAverage(
        dataLogger.barometerFifo, +[](BarometerData& b) { return (double)b.altitude; }, start, len);
}

double ModularFSM::getAccelerationAverage(size_t start, size_t len) {
    return ModularFSM::getAverage(
        dataLogger.highGFifo, +[](HighGData& g) { return (double)g.hg_az; }, start, len);
}

bool ModularFSM::idleEventCheck(){
    if (highG.getAccel().az > launch_linear_acceleration_thresh) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_BOOST;
        return true;
    }
    return false;
}

bool ModularFSM::idleStateCheck(){
    // vel subject to change pending derivative calculations
    float vel = getAltitudeAverage(0, 3) - getAltitudeAverage(3, 3);

    bool altitude_in_range = (launch_site_altitude - alt_error) <= barometer.getAltitude() && barometer.getAltitude() <= (launch_site_altitude + alt_error);
    bool acc_in_range = (1 - acc_error) <= highG.getAccel().az && highG.getAccel().az <= (1 + acc_error);
    bool ang_in_range_pitch = (ang_start - ang_error) <= orientation.getEuler().pitch && orientation.getEuler().pitch <= (ang_start + ang_error);
    bool ang_in_range_yaw = (ang_start - ang_error) <= orientation.getEuler().yaw && orientation.getEuler().yaw <= (ang_start + ang_error);
    bool vel_in_range = -vel_error <= vel && vel <= vel_error;

    if (altitude_in_range && acc_in_range && ang_in_range_pitch && ang_in_range_yaw && vel_in_range) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_IDLE;
        return true;
    }

    rocket_state_ = FSM_State::STATE_UNKNOWN;
    return false;
}

bool ModularFSM::boostEventCheck(){
    if(highG.getAccel().az < boost_to_coast_acceleration){
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_COAST_PREGNC;
        return true;
    }
    return false;
}

bool ModularFSM::boostStateCheck(){
    float vel = getAltitudeAverage(0, 3) - getAltitudeAverage(3, 3);


    bool altitude_in_range = barometer.getAltitude() > launch_site_altitude + alt_error;
    bool acc_in_range = getAccelerationAverage(0,6) > boost_acc_thresh;
    bool ang_in_range_pitch = -boost_ang_thresh <= orientation.getEuler().pitch && orientation.getEuler().pitch <= boost_ang_thresh;
    bool ang_in_range_yaw = -boost_ang_thresh <= orientation.getEuler().yaw && orientation.getEuler().yaw <= boost_ang_thresh;
    bool vel_in_range = vel > vel_error;

     if (altitude_in_range && acc_in_range && ang_in_range_pitch && ang_in_range_yaw && vel_in_range) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_BOOST;
        return true;
    }

    rocket_state_ = FSM_State::STATE_UNKNOWN;
    return false;
}

bool ModularFSM::coastPreGNCEventCheck(){
    return true;
}

bool ModularFSM::coastPreGNCStateCheck(){
    return true;
}

bool ModularFSM::coastGNCEventCheck(){
    return true;
}

bool ModularFSM::coastGNCStateCheck(){
    return true;
}

bool ModularFSM::apogeeEventCheck(){
    return true;
}

bool ModularFSM::apogeeStateCheck(){
    return true;
}

bool ModularFSM::separationEventCheck(){
    return true;
}

bool ModularFSM::separationStateCheck(){
    return true;
}

bool ModularFSM::drogueEventCheck(){
    if (highG.getAccel().az < drogue_to_main_acceleration) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_MAIN;
        return true;
    }
    return false;
}

bool ModularFSM::drogueStateCheck(){
    float velocity = getAltitudeAverage(0, 3) - getAltitudeAverage(3, 3);

    bool altitude_in_range = (launch_site_altitude < barometer.getAltitude()) && (barometer.getAltitude() < apogee_altitude_);
    bool acceleration_in_range = (drogue_ang_acc_bottom < getAccelerationAverage(0,6)) && (getAccelerationAverage(0,6) < drogue_ang_acc_top);
    bool velocity_in_range = velocity < vel_error;
    bool ang_in_range_pitch = drogue_ang_thresh_bottom <= orientation.getEuler().pitch && orientation.getEuler().pitch <= drogue_ang_thresh_top;
    bool ang_in_range_yaw = drogue_ang_thresh_bottom <= orientation.getEuler().yaw && orientation.getEuler().yaw <= drogue_ang_thresh_top;

    if (altitude_in_range && acceleration_in_range && velocity_in_range && ang_in_range_pitch && ang_in_range_yaw) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_DROGUE;
        return true;
    }

    rocket_state_ = FSM_State::STATE_UNKNOWN;
    return false;
}

bool ModularFSM::mainEventCheck(){
    float velocity = getAltitudeAverage(0, 3) - getAltitudeAverage(3, 3);

    if (-vel_error < velocity && velocity < vel_error) {
        last_state_ = FSM_State::STATE_LANDED;
        return true;
    }
    return false;
}

bool ModularFSM::mainStateCheck(){
    float velocity = getAltitudeAverage(0, 3) - getAltitudeAverage(3, 3);

    bool altitude_in_range = (launch_site_altitude < barometer.getAltitude()) && (barometer.getAltitude() < apogee_altitude_);
    bool acceleration_in_range = (getAccelerationAverage(0,6) < main_ang_acc_top);
    bool velocity_in_range = velocity < vel_error;
    bool ang_in_range_pitch = main_ang_thresh_bottom <= orientation.getEuler().pitch && orientation.getEuler().pitch <= main_ang_thresh_top;
    bool ang_in_range_yaw = main_ang_thresh_bottom <= orientation.getEuler().yaw && orientation.getEuler().yaw <= main_ang_thresh_top;

    if (altitude_in_range && acceleration_in_range && velocity_in_range && ang_in_range_yaw && ang_in_range_pitch) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_MAIN;
        return true;
    }
    return false;
}

bool ModularFSM::landedStateCheck(){
    float velocity = getAltitudeAverage(0, 3) - getAltitudeAverage(3, 3);

    bool altitude_in_range = (launch_site_altitude - alt_error < barometer.getAltitude()) && (barometer.getAltitude() < alt_error + launch_site_altitude);
    bool acceleration_in_range = (1 - acc_error < getAccelerationAverage(0,6)) && (getAccelerationAverage(0,6) < 1 + acc_error);
    bool velocity_in_range = (-vel_error < velocity) && (velocity < vel_error);
    
    if (altitude_in_range && acceleration_in_range && velocity_in_range) {
        last_state_ = rocket_state_;
        rocket_state_ = FSM_State::STATE_LANDED;
        return true;
    }
    rocket_state_ = FSM_State::STATE_UNKNOWN;
    return false;
}

void ModularFSM::tickFSM(){
    //lock mutexes
    chMtxLock(&orientation.mutex);
    chMtxLock(&highG.mutex);
    chMtxLock(&barometer.mutex);

    switch(rocket_state_){
        //include a case for init?
        // cheeky bloke init

        case FSM_State::STATE_IDLE:

            if(last_state_ == FSM_State::STATE_IDLE){
                if(!idleEventCheck()){
                    idleStateCheck();
                }
            }
            else{
                if(idleStateCheck()){
                    idleEventCheck();
                }
            }
            break;

        case FSM_State::STATE_BOOST:

            if (last_state_ == FSM_State::STATE_BOOST){
                if(!boostEventCheck()){
                    boostStateCheck();
                }
            }
            else {
                if(boostStateCheck()){
                    boostEventCheck();
                }
            }
            break;

        case FSM_State::STATE_COAST_PREGNC:

            if (last_state_ == FSM_State::STATE_COAST_PREGNC){
                if(!coastPreGNCEventCheck()){
                    coastPreGNCStateCheck();
                }
            }
            else {
                //log time at which we enter coast
                coast_time_ = chVTGetSystemTime();

                if(coastPreGNCStateCheck()){
                    coastPreGNCEventCheck();
                }
            }
            break;

        case FSM_State::STATE_COAST_GNC:

            if(last_state_ == FSM_State::STATE_COAST_GNC){
                if(!coastGNCEventCheck()){
                    coastGNCStateCheck();
                }
            }
            else{
                if(coastGNCStateCheck()){
                    coastGNCEventCheck();
                }
            }
            break;

        case FSM_State::STATE_APOGEE:

            if (last_state_ == FSM_State::STATE_APOGEE){
                //record our apogee
                apogee_altitude_ = barometer.getAltitude();

                if (!apogeeEventCheck()){
                    apogeeStateCheck();
                }
            }
            else{
                if(apogeeStateCheck()){
                    apogeeEventCheck();
                }
            }
            break;

        case FSM_State::STATE_SEPARATION:

            if(last_state_ == FSM_State::STATE_SEPARATION){
                if(!separationEventCheck()){
                    separationStateCheck();
                }
            }
            else{
                if(separationStateCheck()){
                    separationEventCheck();
                }
            }
            break;

        case FSM_State::STATE_DROGUE:

            if(last_state_ == FSM_State::STATE_DROGUE){
                if(!drogueEventCheck()){
                    drogueStateCheck();
                }
            }
            else{
                if(drogueStateCheck()){
                    drogueEventCheck();
                }
            }
            break;

        case FSM_State::STATE_MAIN:

            if(last_state_ == FSM_State::STATE_MAIN){
                if(!mainEventCheck()){
                    mainStateCheck();
                }
            }
            else{
                if(mainStateCheck()){
                    mainEventCheck();
                }
            }
            break;

        case FSM_State::STATE_LANDED:
        
            landedStateCheck();
            break;

        case FSM_State::STATE_UNKNOWN:
            
            if(last_state_ == FSM_State::STATE_IDLE || last_state_ == FSM_State::STATE_BOOST){
			    if(idleStateCheck()){
				    break;
			    }
		    }

            else{
                if(separationStateCheck()){
                    break;
                }
            }

            boostStateCheck() || coastGNCStateCheck() || apogeeStateCheck() 
            || drogueStateCheck() || mainStateCheck() || landedStateCheck();		
            break;

        default:
            break;
    }

    //unlock mutexes used
    chMtxUnlock(&highG.mutex);
    chMtxUnlock(&barometer.mutex);
    chMtxUnlock(&orientation.mutex);
}