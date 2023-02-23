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
    return true;
}

bool ModularFSM::idleStateCheck(){
    return true;
}

bool ModularFSM::boostEventCheck(){
    return true;
}

bool ModularFSM::boostStateCheck(){
    return true;
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
    return true;
}

bool ModularFSM::drogueStateCheck(){
    return true;
}

bool ModularFSM::mainEventCheck(){
    return true;
}

bool ModularFSM::mainStateCheck(){
    return true;
}

bool ModularFSM::landedStateCheck(){
    return true;
}

void ModularFSM::tickFSM(){
    //lock high G mutex, all other mutexes are locked through buffers

    //TODO: lock orientation mutex
    chMtxLock(&highG.mutex);

    switch(rocket_state_){
        //include a case for init?

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

            if(last_state_ == FSM_State::STATE_BOOST){
                if(!boostEventCheck()){
                    boostStateCheck();
                }
            }
            else{
                if(boostStateCheck()){
                    boostEventCheck();
                }
            }
            break;

        case FSM_State::STATE_COAST_PREGNC:

            if(last_state_ == FSM_State::STATE_COAST_PREGNC){
                if(!coastPreGNCEventCheck()){
                    coastPreGNCStateCheck();
                }
            }
            else{
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

            if(last_state_ == FSM_State::STATE_APOGEE){
                if(!apogeeEventCheck()){
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
}