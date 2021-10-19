#include "Eigen30.h"

#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

class ActiveControl {
    public:
        ActiveControl(struct pointers* pointer_struct);

        bool ActiveControl_ON();

        //Frequency should match output of sensor ready (angular velocity)
        // AV_X is x angular acceleration (roll) from LOWG IMU
        void acTickFunction();

    private:
        //These matrices are solely for October launch, only roll control
        Eigen::Matrix<float, 2, 1> k_p {
            0.003,
            -0.003
        };

        Eigen::Matrix<float, 2, 1> k_i {
            .0000007,
            -.0000007
        };

        Eigen::Matrix<float, 2, 1> k_d {
            .0000007,
            -.0000007
        };

        float* gx;
        //goals
        float omega_goal = 0;
        float e_sum = 0;
        float e_prev = 0;
        FSM_State* current_state;
        
};

