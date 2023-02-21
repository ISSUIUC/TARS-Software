#pragma once

#include "mcu_main/finite-state-machines/RocketFSMBase.h"

class ModularFSM : public RocketFSMBase {
    public:
        ModularFSM() = default;

        void tickFSM() override;

    private:
        //coast time should always be set before we look at it
        systime_t coast_time_ = 0;

        //we want a default very high number in case we never reach our apogee
        float apogee_altitude_ = 15000.0;
    
        bool idleEventCheck();
        bool idleStateCheck();

        bool boostEventCheck();
        bool boostStateCheck();

        bool coastPreGNCEventCheck();
        bool coastPreGNCStateCheck();

        bool coastGNCEventCheck();
        bool coastGNCStateCheck();

        bool apogeeEventCheck();
        bool apogeeStateCheck();

        bool separationEventCheck();
        bool separationStateCheck();

        bool drogueEventCheck();
        bool drogueStateCheck();

        bool mainEventCheck();
        bool mainStateCheck();

        bool landedStateCheck();

};