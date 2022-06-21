#include <rk4.h>
#include <PWMServo.h>
#include "dataLog.h"
#include "ServoControl.h"
#include <math.h>
#include <array>

using std::array;

class Controller {
    public:
    void ctrlTickFunction();
    bool ActiveControl_ON();
    Controller(struct pointers* pointer_struct, PWMServo* twisty_boi);

    void setLaunchPadElevation();
    
    PWMServo* twisty_boi_;
    mutex_t* dataMutex_state_;
    stateData* stateData_;
    rk4 rk4_;
    float kp = 0.000042;
    float apogee_des = 4572;
    float min_extension = 0;
    float max_extension = 17.88 / 1000;
    float dt = .006;
    float prev_u = 0;
    float du_max = 0.01;
    float flap_width = 35.1 / 1000; // m

    float launch_pad_alt;
    float apogee_des_msl;
    float apogee_des_agl;

    float* b_alt;
    mutex_t* dataMutex_barometer_;

    FSM_State* current_state;
    ServoControl activeControlServos;
    uint32_t* ac_coast_timer;
};