#include "../eigen-3.4.0/Eigen/Core"
#include "../eigen-3.4.0/Eigen/Dense"

#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"

using namespace Eigen;

typedef struct active_control_data {
    float l1_ = 0;
    float l2_ = 0;
} active_control_data;

struct active_control_data active_control_data_;

class ActiveControl {
    public:
        ActiveControl(struct pointers* pointer_struct);

        //Frequency should match output of sensor ready (angular velocity)
        // AV_X is x angular acceleration (roll) from LOWG IMU
        void acTickFunction();

    private:
        //These matrices are solely for October launch, only roll control
        Matrix<float, 2, 1> k_p {
            {0.003},
            {-0.003}
        };

        Matrix<float, 2, 1> k_i {
            {.0000007},
            {-.0000007}
        };

        Matrix<float, 2, 1> k_d {
            {.0000007},
            {-.0000007}
        };
        float* gx;
        //goals
        float omega_goal = 0;
        float e_sum = 0;
        float e_prev = 0;
        
};

