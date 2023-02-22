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

bool idleEventCheck(){
    return true;
}

bool idleStateCheck(){
    return true;
}

bool boostEventCheck(){
    return true;
}

bool boostStateCheck(){
    return true;
}

bool coastPreGNCEventCheck(){
    return true;
}

bool coastPreGNCStateCheck(){
    return true;
}

bool coastGNCEventCheck(){
    return true;
}

bool coastGNCStateCheck(){
    return true;
}

bool apogeeEventCheck(){
    return true;
}

bool apogeeStateCheck(){
    return true;
}

bool separationEventCheck(){
    return true;
}

bool separationStateCheck(){
    return true;
}

bool drogueEventCheck(){
    return true;
}

bool drogueStateCheck(){
    return true;
}

bool mainEventCheck(){
    return true;
}

bool mainStateCheck(){
    return true;
}

bool landedStateCheck(){
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