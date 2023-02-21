#include "mcu_main/finite-state-machines/ModularFSM.h"
#include "mcu_main/finite-state-machines/thresholds.h"
#include "mcu_main/sensors/sensors.h"

void ModularFSM::tickFSM(){
    
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
