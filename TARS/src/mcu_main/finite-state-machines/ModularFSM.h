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

        //helps determine which checks to run
        FSM_State previous_state_ = FSM_State::STATE_IDLE;

        //used to average data
        double getAltitudeAverage(size_t start, size_t len);
        double getAccelerationAverage(size_t start, size_t len);

};