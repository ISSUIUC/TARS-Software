#ifndef TEMPLATE_FSM_H
#define TEMPLATE_FSM_H


struct pointers;

class TemplateFSM {
   public:
    enum class FSM_State {
        STATE_INIT,
        STATE_IDLE,
        STATE_LAUNCH_DETECT,
        STATE_BOOST,
        STATE_BURNOUT_DETECT,
        STATE_COAST,
        STATE_APOGEE_DETECT,
        STATE_APOGEE,
        STATE_DROGUE_DETECT,
        STATE_DROGUE,
        STATE_MAIN_DETECT,
        STATE_MAIN,
        STATE_LANDED_DETECT,
        STATE_LANDED,
        STATE_ABORT
    };

    TemplateFSM(pointers *ptr);
    void tickFSM();
    FSM_State getFSMState() const { return rocket_state_; }

    private:
    FSM_State rocket_state_;
};

#endif