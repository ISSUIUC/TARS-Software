#ifndef TEMPLATE_FSM_H
#define TEMPLATE_FSM_H

#include "rocketFSM.h"

struct pointers;

class TemplateFSM : public RocketFSM {
   public:
    TemplateFSM(pointers *ptr);
    virtual void tickFSM() override;
};

#endif