#pragma once

#include <ChRt.h>

#include "mcu_main/error.h"
#include "mcu_main/hilsim/HILSIMPacket.h"

class GenericSensor {
    public:
        virtual ErrorCode __attribute__((warn_unused_result)) initialize();
        virtual void update();
        virtual void update(HILSIMPacket hilsim_packet);
};