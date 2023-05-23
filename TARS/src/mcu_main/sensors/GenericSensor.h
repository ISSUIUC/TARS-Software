#pragma once

#include <ChRt.h>

#include "mcu_main/error.h"
#include "mcu_main/hilsim/HILSIMPacket.h"

class GenericSensor {
    public:
        ErrorCode __attribute__((warn_unused_result)) init();
        void update();
        void update(HILSIMPacket hilsim_packet);
        
    private:
};