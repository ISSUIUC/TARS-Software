#include "../EigenArduino-Eigen30/Eigen30.h"
#include "RocketFSMBase.h"
#include "ServoControl.h"
#include "dataLog.h"
#include "rk4.h"
#include "sensors.h"

class KalmanFilter;
extern KalmanFilter kalmanFilter;

struct KalmanState {
    float x;
    float vx;
    float ax;
};

class KalmanFilter {
   public:
    MUTEX_DECL(mutex);

    void Initialize();
    void initialize(float pos_f, float vel_f);
    void priori();
    void update();
    void SetQ(float dt, float sd);
    void SetF(float dt);

    void kfTickFunction(float dt, float sd);

    KalmanState getState() const;
    void updateApogee(float estimate);

   private:
    float s_dt = 0.050;

    float kalman_x = 0;
    float kalman_vx = 0;
    float kalman_ax = 0;
    float kalman_apo = 0;
    systime_t timestamp = 0;

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