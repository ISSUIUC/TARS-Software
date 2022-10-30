/**
 * @file kalmanFilter.cpp
 *
 * @brief Implementation of the AC team's Kalman Filter
 */

#include "kalmanFilter.h"
#include "rocketFSM.h"

#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

/**
 * @brief Run Kalman filter calculations as long as FSM has passed IDLE
 *
 */
void KalmanFilter::kfTickFunction() {
    if (getActiveFSM().getFSMState() > RocketFSMBase::FSM_State::STATE_IDLE) {
        priori();
        update();
    }
}
/**
 * @brief Sets altitude by averaging 30 barometer measurements taken 100 ms
 * apart
 *
 * The following for loop takes a series of barometer measurements on start
 * up and takes the average of them in order to initialize the kalman filter
 * to the correct initial barometric altitude. This is done so that the
 * kalman filter takes minimal time to converge to an accurate state
 * estimate. This process is significantly faster than allowing the state as
 * letting the filter to converge to the correct state can take up to 3 min.
 * This specific process was used because the barometric altitude will
 * change depending on the weather and thus, the initial state estimate
 * cannot be hard coded. A GPS altitude may be used instead but due to GPS
 * losses during high speed/high altitude flight, it is inadvisable with the
 * current hardware to use this as a solution. Reference frames should also
 * be kept consistent (do not mix GPS altitude and barometric).
 *
 */
void KalmanFilter::Initialize() {
    // TODO: The altitude initialization is the same code as
    //   setLaunchPadElevation() in AC. Maybe use the same one?
    float sum = 0;
    for (int i = 0; i < 30; i++) {
        // TODO This mutex lock is almost certainly not necessary
        chMtxLock(&barometer.mutex);
        sum += barometer.getAltitude();
        chMtxUnlock(&barometer.mutex);
        chThdSleepMilliseconds(100);
    }

    // set x_k
    x_k(0, 0) = sum / 30;
    // x_k(0,0) = 1401;
    x_k(1, 0) = 0;
    x_k(2, 0) = 0;

    // set F
    F_mat(0, 1) = s_dt;
    F_mat(0, 2) = (s_dt * s_dt) / 2;
    F_mat(1, 2) = s_dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;
    F_mat(2, 2) = 1;

    // set H
    H(0, 0) = 1;
    H(1, 2) = 1;

    // set P_k
    P_k(0, 0) = 0;
    P_k(0, 1) = 0;
    P_k(0, 2) = 0;
    P_k(1, 1) = 0;
    P_k(2, 2) = 0;
    P_k(1, 2) = 0;
    P_k(2, 1) = P_k(1, 2);
    P_k(1, 0) = P_k(0, 1);
    P_k(2, 0) = P_k(0, 2);
    P_k(1, 0) = P_k(0, 1);

    // set Q
    Q(0, 0) = pow(s_dt, 5) / 20;
    Q(0, 1) = (pow(s_dt, 4) / 8 * 80);
    Q(0, 2) = pow(s_dt, 3) / 6;
    Q(1, 1) = pow(s_dt, 3) / 8;
    Q(1, 2) = pow(s_dt, 2) / 2;
    Q(2, 2) = s_dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);

    // float scale_fact = 75.19;
    // float scale_fact = 14.25;
    float scale_fact = 13.0;
    // float scale_fact = 13;
    Q = Q * scale_fact;

    // set R
    R(0, 0) = 2.0;
    R(1, 1) = 0.01;
    // R(0,0) = 2.;
    // R(1,1) = .01;

    // set B
    B(2, 0) = -1;
}

void KalmanFilter::Initialize(float pos_f, float vel_f) {
    // set x_k
    x_k(0, 0) = pos_f;
    x_k(1, 0) = vel_f;

    // set F
    F_mat(0, 1) = s_dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;

    // set H
    H(0, 0) = 1;

    // set R
    R(0, 0) = 12;

    // set B
    B(2, 0) = -1;
}

void KalmanFilter::priori() {
    // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B
    // or u is = 0
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

/**
 * @brief Update Kalman Gain
 *
 */
void KalmanFilter::update() {
    Eigen::Matrix<float, 2, 2> temp = Eigen::Matrix<float, 2, 2>::Zero();
    temp = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 3, 3> identity = Eigen::Matrix<float, 3, 3>::Identity();
    K = (P_priori * H.transpose()) * temp;

    // TODO These mutex locks are almost certainly not necessary
    // Sensor Measurements
    chMtxLock(&highG.mutex);
    y_k(1, 0) = highG.getAccel().az * 9.81;
    chMtxUnlock(&highG.mutex);

    chMtxLock(&barometer.mutex);
    y_k(0, 0) = barometer.getAltitude();
    chMtxUnlock(&barometer.mutex);

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    chMtxLock(&mutex);
    kalman_x = x_k(0, 0);
    kalman_vx = x_k(1, 0);
    kalman_ax = x_k(2, 0);
    timestamp = chVTGetSystemTime();
    chMtxUnlock(&mutex);
    dataLogger.pushKalmanFifo((KalmanData) {kalman_x, kalman_vx, kalman_ax, kalman_apo, timestamp });
}

KalmanState KalmanFilter::getState() const {
    return {kalman_x, kalman_vx, kalman_ax};
}

void KalmanFilter::updateApogee(float estimate) {
    kalman_apo = estimate;
}

KalmanFilter kalmanFilter;