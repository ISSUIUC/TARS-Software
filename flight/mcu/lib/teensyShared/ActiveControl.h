#include <PWMServo.h>
#include <rk4.h>

#include "ServoControl.h"

class Controller;

extern Controller activeController;

class Controller {
   public:
    void ctrlTickFunction();
    bool ActiveControl_ON();
    Controller();
    void init();

    void startup();

    void setLaunchPadElevation();

    PWMServo controller_servo_;
    rk4 rk4_;
    float kp = 0.00008;

    float min_extension = 0.0;
    float max_extension = 17.88 / 1000.0;
    float dt = .006;
    float prev_u = 0;
    float du_max = 0.01;

    float launch_pad_alt{};
    float apogee_des_msl{};
    float apogee_des_agl = 3962;

    ServoControl activeControlServos;
};

