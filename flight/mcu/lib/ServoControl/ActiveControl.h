#include <../eigen-3.4.0/Eigen/Core>
#include <../eigen-3.4.0/Eigen/Dense>

using namespace Eigen;

class ActiveControl {
    public:
        ActiveControl();

        //Frequency should match output of sensor ready (angular velocity)
        // AV_X is x angular acceleration (roll) from LOWG IMU
        void acTickFunction(float AV_X);

    private:
        //These matrices are solely for October launch, only roll control
        Matrix<float, 3, 3> A_oct {
            {0, 1, 0},
            {0, -0.0728169, 0},
            {0, 0, 0}
        };
        Matrix<float, 3, 2> B_oct {
            {0, 0},
            {-49.45616789, -49.45616789},
            {432.68869846, -432.68869846}
        };
        Matrix<float, 2, 3> k_oct {
            {0, 0, 0.005},
            {0, 0, -0.005}
        };

        //These matrices are for future full-state estimation purposes. Keep here for stress testing?
        Matrix<float, 16, 16> k; 
        Matrix<float, 16, 16> A;
        Matrix<float, 16, 16> B;

};