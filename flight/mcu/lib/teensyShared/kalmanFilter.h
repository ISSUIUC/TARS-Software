#include <deque>

#include "../EigenArduino-Eigen30/Eigen30.h"
#include "ServoControl.h"
#include "dataLog.h"
#include "rk4.h"
#include "rocketFSM.h"
#include "sensors.h"
class KalmanFilter {
   public:
    KalmanFilter(struct pointers* pointer_struct);

    void Initialize();
    void Initialize(float pos_f, float vel_f);
    void UpdateF(float dt);
    void UpdateQ(float dt, float spectral_density);
    void priori(float dt, float spectral_density);
    void update();

    void kfTickFunction(float dt, float spectral_density);
    void tickBuffer();

    float bufferAverage();
    Eigen::Matrix<float, 3, 1> getStateData();
    void setStateData(Eigen::Matrix<float, 3, 1> state_data);

   private:
    float s_dt = 0.050;

    FifoBuffer<float, 15>
        b_alt_buffer;  // 2400 is the max number of samples in 2min given a 50ms
                       // delay between samples
    DataLogBuffer* data_logger_;
    mutex_t* mutex_lowG_;
    mutex_t* mutex_highG_;
    mutex_t* dataMutex_barometer_;
    mutex_t* dataMutex_state_;
    stateData* stateData_;
    RocketFSM::FSM_State* current_state_;
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
    Eigen::Matrix<float, 2, 1> y_k = Eigen::Matrix<float, 2, 1>::Zero();

    Eigen::Matrix<float, 3, 2> B = Eigen::Matrix<float, 3, 2>::Zero();
};
