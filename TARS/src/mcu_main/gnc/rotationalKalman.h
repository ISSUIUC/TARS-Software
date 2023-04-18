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

class RotationalKalmanFilter;
extern RotationalKalmanFilter rotationalKalmanFilter;

class RotationalKalmanFilter : public KalmanFilter<9, 6>{
    public:
        MUTEX_DECL(mutex);

        RotationalKalmanFilter();

        void Initialize() override;
        void priori() override;
        void update() override;

        void SetQ(float dt, float sd) override;
        void SetF(float dt) override;

        KalmanState getState() const override;
        void setState(KalmanState state) override;

       private:
        float s_dt = 0.050;
        float rqf = 1;

        KalmanState rotational_kalman_state;
};