#include "../EigenArduino-Eigen30/Eigen30.h"
#include "ServoControl.h"
#include "dataLog.h"
#include "rk4.h"
#include "rocketFSM.h"
#include "sensors.h"
#include "OrientationSensor.h"

class KalmanFilter {
   public:
    KalmanFilter(struct pointers* pointer_struct);

    void Initialize();
    void Initialize(float pos_f, float vel_f);
    void priori();
    void update();
    void SetQ(float dt, float sd);
    void SetF(float dt);
    Eigen::Matrix<float, 9, 1> BodyToGlobal(Eigen::Matrix<float, 9, 1> & x_k, euler_t angles);

    void kfTickFunction(float dt, float sd);

   private:
    float s_dt = 0.050;

    DataLogBuffer* data_logger_;
    mutex_t* mutex_lowG_;
    mutex_t* mutex_highG_;
    mutex_t* dataMutex_barometer_;
    mutex_t* dataMutex_state_;
    stateData* stateData_;
    RocketFSM::FSM_State* current_state_;
    float* b_alt;
    float* gz_L;
    float* ax_H;
    float* ay_H;
    float* az_H;

    Eigen::Matrix<float, 9, 1> x_k = Eigen::Matrix<float, 9, 1>::Zero();
    Eigen::Matrix<float, 9, 9> F_mat = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 4, 9> H = Eigen::Matrix<float, 4, 9>::Zero();
    Eigen::Matrix<float, 9, 9> P_k = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 9, 9> Q = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 4, 4> R = Eigen::Matrix<float, 4, 4>::Zero(); // Diagonal
    Eigen::Matrix<float, 9, 9> P_priori = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 9, 1> x_priori = Eigen::Matrix<float, 9, 1>::Zero();
    Eigen::Matrix<float, 9, 4> K = Eigen::Matrix<float, 9, 4>::Zero();
    Eigen::Matrix<float, 4, 1> y_k = Eigen::Matrix<float, 4, 1>::Zero();

    Eigen::Matrix<float, 9, 4> B = Eigen::Matrix<float, 9, 4>::Zero();
};
