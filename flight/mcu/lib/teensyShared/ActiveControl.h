#include "../eigen-3.4.0/Eigen/Core"
#include "../eigen-3.4.0/Eigen/Dense"

using namespace Eigen;

class ActiveControl {
   public:
    ActiveControl();

<<<<<<< HEAD
        bool ActiveControl_ON();

        //Frequency should match output of sensor ready (angular velocity)
        // AV_X is x angular acceleration (roll) from LOWG IMU
        void acTickFunction();

    private:
        
        mutex_t* mutex_lowG_;
        
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

        float l1_prev = 0;
        float l2_prev = 0;
        float* gx;
        float du_max = .0001; //TODO test value
        float dt = .006; //seconds

        //goals
        float omega_goal = 0;
        float e_sum = 0;
        float e_prev = 0;
        FSM_State* current_state;
        ServoControl activeControlServos;
        struct pointers* pointers;
};
=======
    // Frequency should match output of sensor ready (angular velocity)
    // AV_X is x angular acceleration (roll) from LOWG IMU
    void acTickFunction(float AV_X);
>>>>>>> 050ec086108d4593c8c46c0da8219e4543ef29ac

   private:
    // These matrices are solely for October launch, only roll control
    Matrix<float, 2, 1> k_oct{{0.005}, {-0.005}};
};