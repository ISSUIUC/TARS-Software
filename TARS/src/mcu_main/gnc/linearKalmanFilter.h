#pragma once

#include "../EigenArduino-Eigen30/Eigen30.h"
#include "common/FifoBuffer.h"
#include "common/ServoControl.h"
#include "common/packet.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/finite-state-machines/RocketFSMBase.h"
#include "mcu_main/gnc/rk4.h"
#include "mcu_main/gnc/kalmanFilter.h"
#include "mcu_main/sensors/sensors.h"

#define BUFFER_BIGGNESS 10

class LinearKalmanFilter;
extern LinearKalmanFilter linearKalmanFilter;

class LinearKalmanFilter : public KalmanFilter<9, 4>{
   public:
    MUTEX_DECL(mutex);
    LinearKalmanFilter();
    void Initialize() override;
    void Initialize(float pos_x, float vel_x, float pos_y, float vel_y, float pos_z, float vel_z);
    void priori() override;
    void update() override;

    void SetQ(float dt, float sd) override;
    void SetF(float dt) override;

    KalmanState getState() const override;
    void setState(KalmanState state) override;
    void updateApogee(float estimate);

   private:
    float s_dt = 0.050;

    KalmanState kalman_state;
    float kalman_apo = 0;
    systime_t timestamp = 0;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Eigen::Matrix<float, 3, 1> world_accel;

    FifoBuffer<float, BUFFER_BIGGNESS> alt_buffer;
};
