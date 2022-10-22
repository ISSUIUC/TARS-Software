#include <PWMServo.h>
#include <math.h>
#include <rk4.h>

#include <array>

#include "ServoControl.h"
#include "dataLog.h"

using std::array;

class Controller {
   public:
    void ctrlTickFunction(pointers* pointer_struct);
    bool ActiveControl_ON();
    Controller(struct pointers* pointer_struct, PWMServo* controller_servo);

    void setLaunchPadElevation();

    PWMServo* controller_servo_;
    mutex_t* dataMutex_state_;
    stateData* stateData_;
    rk4 rk4_;
    float kp = 0.00008;

    float min_extension = 0;
    float max_extension = 17.88 / 1000;
    float dt = .006;
    float prev_u = 0;
    float du_max = 0.01;

    float launch_pad_alt;
    float apogee_des_msl;
    float apogee_des_agl = 9144;

    float* b_alt;
    mutex_t* dataMutex_barometer_;

    RocketFSM::FSM_State* current_state;
    ServoControl activeControlServos;
};