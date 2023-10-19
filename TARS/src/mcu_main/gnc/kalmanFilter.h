#pragma once

#include "mcu_main/ServoControl.h"
#include "common/FifoBuffer.h"
#include "common/packet.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/finite-state-machines/RocketFSMBase.h"

#include "mcu_main/sensors/sensors.h"

#include "mcu_main/gnc/rk4.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

class KalmanFilter;
extern KalmanFilter kalmanFilter;
extern KalmanFilter rotational_kalmanFilter;

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
} KalmanState;

class KalmanFilter {
   public:
    MUTEX_DECL(mutex);

    void Initialize();
    void Initialize(float pos_x, float vel_x, float pos_y, float vel_y, float pos_z, float vel_z);
    void priori();
    void update();

    void SetQ(float dt, float sd);
    void SetF(float dt);
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

    void kfTickFunction(float dt, float sd);

    KalmanState getState() const;
    void setState(KalmanState state);
    void updateApogee(float estimate);

   private:
    bool rotational;
    float s_dt = 0.050;

    KalmanState kalman_state;
    float kalman_apo = 0;
    systime_t timestamp = 0;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Eigen::Matrix<float, 3, 1> world_accel;

    FifoBuffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;

    Eigen::Matrix<float, NUM_STATES, 1> x_k = Eigen::Matrix<float, NUM_STATES, 1>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> F_mat = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_STATES> H = Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> P_k = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> Q = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_SENSOR_INPUTS> R =
        Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_SENSOR_INPUTS>::Zero();  // Diagonal
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> P_priori = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_STATES, 1> x_priori = Eigen::Matrix<float, NUM_STATES, 1>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS> K = Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS>::Zero();
    Eigen::Matrix<float, NUM_SENSOR_INPUTS, 1> y_k = Eigen::Matrix<float, NUM_SENSOR_INPUTS, 1>::Zero();

    Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS> B = Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS>::Zero();
};
