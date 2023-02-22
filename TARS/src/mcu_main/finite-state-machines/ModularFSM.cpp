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
            break;

        case FSM_State::STATE_BOOST:
            break;

        case FSM_State::STATE_COAST_PREGNC:
            break;

        case FSM_State::STATE_COAST_GNC:
            break;

        case FSM_State::STATE_APOGEE:
            break;

        case FSM_State::STATE_SEPARATION:
            break;

        case FSM_State::STATE_DROGUE:
            break;

        case FSM_State::STATE_MAIN:
            break;

        case FSM_State::STATE_LANDED:
            break;

        case FSM_State::STATE_UNKNOWN:
            break;

        default:
            break;
    }
}