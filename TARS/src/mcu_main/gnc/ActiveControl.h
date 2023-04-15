#include <mcu_main/gnc/rk4.h>
#include "mcu_main/ServoControl.h"

#ifndef ENABLE_SILSIM_MODE
#include <PWMServo.h>
#endif

class Controller;

extern Controller activeController;

class Controller {
   public:
    void ctrlTickFunction();
    bool ActiveControl_ON();
    Controller();
    void init();

    void setLaunchPadElevation();

#ifndef ENABLE_SILSIM_MODE
    PWMServo controller_servo_;
#endif
    rk4 rk4_;
    float kp = 0.0002;

    float min_extension = 0.0;
    float max_extension = 12.5 / 1000.0;
    float dt = .006;
    float prev_u = 0;
    float du_max = 0.01;

    float launch_pad_alt = 0.0;
    float apogee_des_msl = 0.0;
    float apogee_des_agl = 4000;

    ServoControl activeControlServos;
};
