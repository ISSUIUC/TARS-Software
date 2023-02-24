#pragma once

#include "../EigenArduino-Eigen30/Eigen30.h"
#include "common/ServoControl.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/finite-state-machines/RocketFSMBase.h"
#include "mcu_main/gnc/rk4.h"
#include "mcu_main/sensors/sensors.h"
#include "common/packet.h"

class KalmanFilter;
extern KalmanFilter kalmanFilter;

typedef struct KalmanState {
    float state_est_pos_x;
    float state_est_vel_x;
    float state_est_accel_x;
    float state_est_pos_y;
    float state_est_vel_y;
    float state_est_accel_y;
    float state_est_pos_z;
    float state_est_vel_z;
    float state_est_accel_z;

    float state_est_r_pos_x;
    float state_est_r_vel_x;
    float state_est_r_accel_x;
    float state_est_r_pos_y;
    float state_est_r_vel_y;
    float state_est_r_accel_y;
    float state_est_r_pos_z;
    float state_est_r_vel_z;
    float state_est_r_accel_z;
} KalmanState;

class KalmanFilter {
   public:
    MUTEX_DECL(mutex);

    void Initialize();
    void Initialize(float pos_x, float vel_x, float pos_y, float vel_y, float pos_z, float vel_z);
    void priori();
    void update();
    void priori_r();
    void update_r();

    void SetQ(float dt, float sd);
    void SetF(float dt);
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

    void kfTickFunction(float dt, float sd);

    KalmanState getState() const;
    void updateApogee(float estimate);

   private:
    float s_dt = 0.050;

    // float kalman_x = 0;
    // float kalman_vx = 0;
    // float kalman_ax = 0;
    KalmanState kalman_state;
    float kalman_apo = 0;
    systime_t timestamp = 0;

    Eigen::Matrix<float, 9, 1> x_k = Eigen::Matrix<float, 9, 1>::Zero();
    Eigen::Matrix<float, 9, 9> F_mat = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 4, 9> H = Eigen::Matrix<float, 4, 9>::Zero();
    Eigen::Matrix<float, 9, 9> P_k = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 9, 9> Q = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 4, 4> R = Eigen::Matrix<float, 4, 4>::Zero();  // Diagonal
    Eigen::Matrix<float, 9, 9> P_priori = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 9, 1> x_priori = Eigen::Matrix<float, 9, 1>::Zero();
    Eigen::Matrix<float, 9, 4> K = Eigen::Matrix<float, 9, 4>::Zero();
    Eigen::Matrix<float, 4, 1> y_k = Eigen::Matrix<float, 4, 1>::Zero();

    Eigen::Matrix<float, 9, 1> x_k_r = Eigen::Matrix<float, 9, 1>::Zero();
    Eigen::Matrix<float, 6, 9> H_r = Eigen::Matrix<float, 6, 9>::Zero();
    Eigen::Matrix<float, 9, 9> P_k_r = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 6, 6> R_r = Eigen::Matrix<float, 6, 6>::Zero();  // Diagonal
    Eigen::Matrix<float, 9, 9> P_priori_r = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 9, 1> x_priori_r = Eigen::Matrix<float, 9, 1>::Zero();
    Eigen::Matrix<float, 9, 6> K_r = Eigen::Matrix<float, 9, 6>::Zero();
    Eigen::Matrix<float, 6, 1> y_k_r = Eigen::Matrix<float, 6, 1>::Zero();

    Eigen::Matrix<float, 9, 4> B = Eigen::Matrix<float, 9, 4>::Zero();
};
