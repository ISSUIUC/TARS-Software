#include "../EigenArduino-Eigen30/Eigen30.h"
#include "ServoControl.h"
#include "acShared.h"
#include "dataLog.h"
#include "sensors.h"
#include "rk4.h"

class KalmanFilter { 
    public:

    KalmanFilter(struct pointers* pointer_struct);
    
    void Initialize();
    void Initialize(float pos_f, float vel_f);
    void priori();
    void update();

    void kfTickFunction();

    private:

    float s_dt = 0.050;

    DataLogBuffer* data_logger_;
    mutex_t* mutex_lowG_;
    mutex_t* mutex_highG_;
    mutex_t* dataMutex_barometer_;
    mutex_t* dataMutex_state_;
    stateData* stateData_;
    FSM_State* current_state_;
    float* b_alt;
    float* gz_L;
    float* gz_H;

    Eigen::Matrix<float, 3, 1> x_k{0, 0, 0};
    Eigen::Matrix<float, 3, 3> F_mat = Eigen::Matrix<float, 3, 3>::Zero();
    Eigen::Matrix<float, 2, 3> H = Eigen::Matrix<float, 2, 3>::Zero();
    Eigen::Matrix<float, 3, 3> P_k = Eigen::Matrix<float, 3, 3>::Zero();
    Eigen::Matrix<float, 3, 3> Q = Eigen::Matrix<float, 3, 3>::Zero();
    Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero();
    Eigen::Matrix<float, 3, 3> P_priori = Eigen::Matrix<float, 3, 3>::Zero();
    Eigen::Matrix<float, 3, 1> x_priori = Eigen::Matrix<float, 3, 1>::Zero();
    Eigen::Matrix<float, 3, 2> K = Eigen::Matrix<float, 3, 2>::Zero();
    Eigen::Matrix<float, 2, 2> temp = Eigen::Matrix<float, 2, 2>::Zero();
    Eigen::Matrix<float, 2, 1> y_k = Eigen::Matrix<float, 2, 1>::Zero();
    Eigen::Matrix<float, 3, 3> identity = Eigen::Matrix<float, 3, 3>::Identity();

    Eigen::Matrix<float, 3, 2> B = Eigen::Matrix<float, 3, 2>::Zero();
};
