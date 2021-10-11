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
        Matrix<float, 2, 1> k_oct {
            {0.005},
            {-0.005}
        };

        //These matrices are for future full-state estimation purposes. Keep here for stress testing?
        Matrix<float, 16, 16> k; 

};