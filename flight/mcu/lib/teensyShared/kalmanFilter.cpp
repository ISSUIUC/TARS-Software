/**
 * @file kalmanFilter.cpp
 *
 * @brief Implementation of a Linear Kalman Filter to estimate position, velocity, and acceleration.
 *
 * @details This class takes input data from a barometer and accelerometer to estimate state data for the rocket.
 */

#include "kalmanFilter.h"
#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

/**
 * @brief Sets the Q matrix given time step and spectral density.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 *
 * The Q matrix is the covariance matrix for the process noise and is
 * updated based on the time taken per cycle of the Kalman Filter Thread.
 */
void KalmanFilter::SetQ(float dt, float sd) {
    Q(0, 0) = pow(dt, 5) / 20;
    Q(0, 1) = pow(dt, 4) / 8 * 80;
    Q(0, 2) = pow(dt, 3) / 6;
    Q(1, 1) = pow(dt, 3) / 8;
    Q(1, 2) = pow(dt, 2) / 2;
    Q(2, 2) = dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);
    Q *= sd;
}

/**
 * @brief Sets the F matrix given time step.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 *
 * The F matrix is the state transition matrix and is defined
 * by how the states change over time.
 */
void KalmanFilter::SetF(float dt) {
    F_mat(0, 1) = dt;
    F_mat(0, 2) = (s_dt * dt) / 2;
    F_mat(1, 2) = dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;
    F_mat(2, 2) = 1;
}

KalmanFilter::KalmanFilter(struct pointers* pointer_struct) {
    gz_L = &pointer_struct->sensorDataPointer->lowG_data.gz;
    gz_H = &pointer_struct->sensorDataPointer->highG_data.hg_az;
    b_alt = &pointer_struct->sensorDataPointer->barometer_data.altitude;
    mutex_lowG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG;
    mutex_highG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG;
    dataMutex_barometer_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer;
    dataMutex_state_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_state;
    stateData_ = &pointer_struct->sensorDataPointer->state_data;
    data_logger_ = &pointer_struct->dataloggerTHDVarsPointer;
    current_state_ =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketStates[0];  // TODO use all rocket states?
}

/**
 * @brief Run Kalman filter calculations as long as FSM has passed IDLE
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 */
void KalmanFilter::kfTickFunction(float dt, float sd) {
    if (*current_state_ >= RocketFSM::FSM_State::STATE_IDLE) {
        SetF(float(dt) / 1000);
        SetQ(float(dt) / 1000, sd);
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
    // setLaunchPadElevation() in AC. Maybe use the same one?
    float sum = 0;
    for (int i = 0; i < 30; i++) {
        chMtxLock(dataMutex_barometer_);
        sum += *b_alt;
        chMtxUnlock(dataMutex_barometer_);
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
    Q(0, 1) = pow(s_dt, 4) / 8 * 80;
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
    R(1, 1) = 0.1;
    // R(0,0) = 2.;
    // R(1,1) = .01;

    // set B
    B(2, 0) = -1;
}

/**
 * @brief Initializes the Kalman Filter with an initial position and velocity estimate
 *
 * @param pos_f Initial position estimate
 * @param vel_f Initial velocity estimate
 */

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

void KalmanFilter::setState(float pos_f, float vel_f, float accel_f) {
    x_k(0, 0) = pos_f;
    x_k(1, 0) = vel_f;
    x_k(2, 0) = accel_f;
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

void KalmanFilter::priori() {
    // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B
    // or u is = 0
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

/**
 * @brief Update Kalman Gain and state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate
 * and Kalman gain. The Kalman gain can be considered as a measure of how uncertain
 * the new sensor data is. After updating the gain, the state estimate is updated.
 *
 */
void KalmanFilter::update() {
    // chMtxLock(dataMutex_state_);
    if (*current_state_ >= RocketFSM::FSM_State::STATE_APOGEE) {
        H(1, 2) = 0;
    }

    if (*current_state_ == RocketFSM::FSM_State::STATE_LAUNCH_DETECT) {
    // if (TIME_I2MS(chVTGetSystemTime() - joe_mama) > 5000) {
        float cur_reading;
        float sum;
        while (alt_buffer.pop(&cur_reading)) {
            sum += cur_reading;
        }
        sum = sum / 10.0;
        setState(sum, 0, 0);
        // Serial.println("Resetting Kalman Filter");
    }
    // if (*current_state == RocketFSM::FSM_State::STATE_LAUNCH_DETECT) {
    //     float cur_reading;
    //     float sum;
    //     while (alt_buffer.size() > 0) {
    //         alt_buffer.pop(&cur_reading);
    //         sum += cur_reading;
    //     }
    //     sum = sum / 10;
    //     setState(sum, 0, 0);
    // }

    // }
    // chMtxUnlock(dataMutex_state_);
    Eigen::Matrix<float, 2, 2> temp = Eigen::Matrix<float, 2, 2>::Zero();
    temp = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 3, 3> identity = Eigen::Matrix<float, 3, 3>::Identity();
    K = (P_priori * H.transpose()) * temp;

    // Sensor Measurements
    chMtxLock(mutex_highG_);
    y_k(1, 0) = ((*gz_H) * 9.81) - 9.81 - 0.51;
    chMtxUnlock(mutex_highG_);

    chMtxLock(dataMutex_barometer_);
    y_k(0, 0) = *b_alt;
    alt_buffer.push(*b_alt);
    chMtxUnlock(dataMutex_barometer_);

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    chMtxLock(dataMutex_state_);
    stateData_->state_x = x_k(0, 0);
    stateData_->state_vx = x_k(1, 0);
    stateData_->state_ax = x_k(2, 0);
    stateData_->timeStamp_state = chVTGetSystemTime();
    chMtxUnlock(dataMutex_state_);
    data_logger_->pushStateFifo(stateData_);
}
