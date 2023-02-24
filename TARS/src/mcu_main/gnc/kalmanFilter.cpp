/**
 * @file kalmanFilter.cpp
 *
 * @brief Implementation of a Linear Kalman Filter to estimate position, velocity, and acceleration.
 *
 * @details This class takes input data from a barometer and accelerometer to estimate state data for the rocket.
 */

#include "mcu_main/gnc/kalmanFilter.h"

#include "mcu_main/finite-state-machines/rocketFSM.h"

#include <cmath>

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
    for (int i = 0; i < 3; i++) {
        Q(3 * i, 3 * i) = pow(dt, 5) / 20;
        Q(3 * i, 3 * i + 1) = pow(dt, 4) / 8 * 80;
        Q(3 * i, 3 * i + 2) = pow(dt, 3) / 6;
        Q(3 * i + 1, 3 * i + 1) = pow(dt, 3) / 8;
        Q(3 * i + 1, 3 * i + 2) = pow(dt, 2) / 2;
        Q(3 * i + 2, 3 * i + 2) = dt;
        Q(3 * i + 1, 3 * i) = Q(3 * i, 3 * i + 1);
        Q(3 * i + 2, 3 * i) = Q(3 * i, 3 * i + 2);
        Q(3 * i + 2, 3 * i + 1) = Q(3 * i + 1, 3 * i + 2);
    }

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
    for (int i = 0; i < 3; i++) {
        F_mat(3 * i, 3 * i + 1) = s_dt;
        F_mat(3 * i, 3 * i + 1) = (s_dt * s_dt) / 2;
        F_mat(3 * i + 1, 3 * i + 2) = s_dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }
}

/**
 * @brief Run Kalman filter calculations as long as FSM has passed IDLE
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 */
void KalmanFilter::kfTickFunction(float dt, float sd) {
    if (getActiveFSM().getFSMState() > FSM_State::STATE_IDLE) {
        SetF(float(dt) / 1000);
        SetQ(float(dt) / 1000, sd);
        priori();
        priori_r();
        update();
        update_r();
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
    for (int i = 0; i < 3; i++) {
        // set F
        F_mat(3*i, 3*i+1) = s_dt;
        F_mat(3*i, 3*i+1) = (s_dt * s_dt) / 2;
        F_mat(3*i+1, 3*i+2) = s_dt;

        F_mat(3*i, 3*i) = 1;
        F_mat(3*i+1, 3*i+1) = 1;
        F_mat(3*i+2, 3*i+2) = 1;

        // set Q
        Q(3*i, 3*i) = pow(s_dt, 5) / 20;
        Q(3*i, 3*i+1) = pow(s_dt, 4) / 8 * 80;
        Q(3*i, 3*i+2) = pow(s_dt, 3) / 6;
        Q(3*i+1, 3*i+1) = pow(s_dt, 3) / 8;
        Q(3*i+1, 3*i+2) = pow(s_dt, 2) / 2;
        Q(3*i+2, 3*i+2) = s_dt;
        Q(3*i+1, 3*i) = Q(3*i, 3*i+1);
        Q(3*i+2, 3*i) = Q(3*i, 3*i+2);
        Q(3*i+2, 3*i+1) = Q(3*i+1, 3*i+2);
    }

    // F_mat(0, 1) = s_dt;
    // F_mat(0, 2) = (s_dt * s_dt) / 2;
    // F_mat(1, 2) = s_dt;

    // F_mat(0, 0) = 1;
    // F_mat(1, 1) = 1;
    // F_mat(2, 2) = 1;

    // set H
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 5) = 1;
    H(3, 8) = 1;

    H_r(0, 0) = 1;
    H_r(0, 1) = 1;
    H_r(1, 2) = 1;
    H_r(1, 3) = 1;
    H_r(2, 4) = 1;
    H_r(2, 5) = 1;
    

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

    // float scale_fact = 75.19;
    // float scale_fact = 14.25;
    float scale_fact = 13.0;
    // float scale_fact = 13;
    Q = Q * scale_fact;

    // set R
    R(0, 0) = 2.0;
    R(1, 1) = 1.9;
    R(2, 2) = 1.9;
    R(3, 3) = 1.9;
    // R(0,0) = 2.;
    // R(1,1) = .01;

    float x_sum = 0;
    float y_sum = 0;
    float z_sum = 0;

    euler_t orientations;

    for (int i = 0; i < 30; i++) {
        chMtxLock(&orientation.mutex);
        orientations = orientation.getEuler();
        chMtxUnlock(&orientation.mutex);
        x_sum += orientations.roll;
        y_sum += orientations.pitch;
        z_sum += orientations.yaw;
    }

    x_k(0, 0) = x_sum / 30;
    x_k(3, 0) = y_sum / 30;
    x_k(6, 0) = z_sum / 30;

    R_r(0, 0) = 1.9;
    R_r(1, 1) = 1.9;
    R_r(2, 2) = 1.9;
    R_r(3, 3) = 1.9;
    R_r(4, 4) = 1.9;
    R_r(5, 5) = 1.9;
    

    // set B
    B(2, 0) = -1;
}

/**
 * @brief Initializes the Kalman Filter with an initial position and velocity estimate
 *
 * @param pos_x Initial position estimate in the x direction
 * @param vel_x Initial velocity estimate in the x direction
 * @param accel_x Initial acceleration estimate in the x direction
 * @param pos_y Initial position estimate in the y direction
 * @param vel_y Initial velocity estimate in the y direction
 * @param accel_y Initial acceleration estimate in the y direction
 * @param pos_z Initial position estimate in the z direction
 * @param vel_z Initial velocity estimate in the z direction
 * @param accel_z Initial acceleration estimate in the z direction
 */

void KalmanFilter::Initialize(float pos_x, float vel_x,
                              float pos_y, float vel_y,
                              float pos_z, float vel_z) {
    // set x_k
    x_k(0, 0) = pos_x;
    x_k(1, 0) = vel_x;

    x_k(3, 0) = pos_y;
    x_k(4, 0) = vel_y;

    x_k(6, 0) = pos_z;
    x_k(7, 0) = vel_z;


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
    if (getActiveFSM().getFSMState() == FSM_State::STATE_LAUNCH_DETECT) {
        float cur_reading;
        float sum = 0;
        while (alt_buffer.read(cur_reading)) {
            sum += cur_reading;
        }
        sum = sum / 10.0;
        setState((KalmanState){sum, 0, 0, 0, 0, 0, 0, 0, 0});
    }
    
    if (getActiveFSM().getFSMState() >= FSM_State::STATE_APOGEE) {
        H(1, 2) = 0;
    }

    Eigen::Matrix<float, 4, 4> temp = Eigen::Matrix<float, 4, 4>::Zero();
    temp = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * temp;

    // TODO These mutex locks are almost certainly not necessary
    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> accel = Eigen::Matrix<float, 3, 1>::Zero();
    chMtxLock(&highG.mutex);
    accel(0, 0) = highG.getAccel().az;
    accel(1, 0) = highG.getAccel().ax;
    accel(2, 0) = highG.getAccel().ay;
    chMtxUnlock(&highG.mutex);

    chMtxLock(&orientation.mutex);
    euler_t angles = orientation.getEuler();
    chMtxUnlock(&orientation.mutex);

    Eigen::Matrix<float, 3, 1> acc = BodyToGlobal(angles, accel);
    y_k(1, 0) = acc(0) - 9.81;
    y_k(2, 0) = acc(1);
    y_k(3, 0) = acc(2);

    chMtxLock(&barometer.mutex);
    y_k(0, 0) = barometer.getAltitude();
    alt_buffer.push(barometer.getAltitude());
    chMtxUnlock(&barometer.mutex);

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    chMtxLock(&mutex);
    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = x_k(2, 0);
    kalman_state.state_est_pos_y = x_k(3, 0);
    kalman_state.state_est_vel_y = x_k(4, 0);
    kalman_state.state_est_accel_y = x_k(5, 0);
    kalman_state.state_est_pos_z = x_k(6, 0);
    kalman_state.state_est_vel_z = x_k(7, 0);
    kalman_state.state_est_accel_z = x_k(8, 0);

    timestamp = chVTGetSystemTime();
    chMtxUnlock(&mutex);

    struct KalmanData kalman_data;
    kalman_data.kalman_acc_x = kalman_state.state_est_accel_x;
    kalman_data.kalman_acc_y = kalman_state.state_est_accel_y;
    kalman_data.kalman_acc_z = kalman_state.state_est_accel_z;
    kalman_data.kalman_vel_x = kalman_state.state_est_vel_x;
    kalman_data.kalman_vel_y = kalman_state.state_est_vel_y;
    kalman_data.kalman_vel_z = kalman_state.state_est_vel_z;
    kalman_data.kalman_pos_x = kalman_state.state_est_pos_x;
    kalman_data.kalman_pos_y = kalman_state.state_est_pos_y;
    kalman_data.kalman_pos_z = kalman_state.state_est_pos_z;
    kalman_data.kalman_apo = kalman_apo;
    kalman_data.timeStamp_state = timestamp;

    dataLogger.pushKalmanFifo(kalman_data);
}

Eigen::Matrix<float, 3, 1> KalmanFilter::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> body_vect) {
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0., 0., cos(angles.roll), -sin(angles.roll ), 0., sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0., sin(angles.pitch), 0., 1., 0., -sin(angles.pitch), 0., cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0., sin(angles.yaw), cos(angles.yaw), 0., 0., 0., 1.;
    return yaw * pitch * roll * body_vect;
}

void KalmanFilter::priori_r() {
    x_priori_r = (F_mat * x_k_r);
    P_priori_r = (F_mat * P_k_r * F_mat.transpose()) + Q;
}

void KalmanFilter::update_r() {
    Eigen::Matrix<float, 6, 6> temp = Eigen::Matrix<float, 6, 6>::Zero();
    temp = (((H_r * P_priori_r * H_r.transpose()) + R_r)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K_r = (P_priori_r * H_r.transpose()) * temp;

    // TODO These mutex locks are almost certainly not necessary
    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> gyro = Eigen::Matrix<float, 3, 1>::Zero();
    chMtxLock(&lowG.mutex);
    gyro(0, 0) = lowG.getGyroscope().gz;
    gyro(1, 0) = lowG.getGyroscope().gx;
    gyro(2, 0) = lowG.getGyroscope().gy;
    chMtxUnlock(&lowG.mutex);

    euler_t angles;
    angles.roll = x_k_r(0);
    angles.pitch = x_k_r(1);
    angles.yaw = x_k_r(2);
    
    Eigen::Matrix<float, 3, 1> euler = BodyToGlobal(angles, gyro);
    y_k_r(0, 0) = euler(0);
    y_k_r(2, 0) = euler(1);
    y_k_r(4, 0) = euler(2);

    // # Posteriori Update
    x_k_r = x_priori_r + K_r * (y_k_r - (H_r * x_priori_r));
    P_k_r = (identity - K_r * H_r) * P_priori_r;

    chMtxLock(&mutex);
    kalman_state.state_est_r_pos_x = x_k_r(0, 0);
    kalman_state.state_est_r_vel_x = x_k_r(1, 0);
    kalman_state.state_est_r_accel_x = x_k_r(2, 0);
    kalman_state.state_est_r_pos_y = x_k_r(3, 0);
    kalman_state.state_est_r_vel_y = x_k_r(4, 0);
    kalman_state.state_est_r_accel_y = x_k_r(5, 0);
    kalman_state.state_est_r_pos_z = x_k_r(6, 0);
    kalman_state.state_est_r_vel_z = x_k_r(7, 0);
    kalman_state.state_est_r_accel_z = x_k_r(8, 0);

    chMtxUnlock(&mutex);

    struct KalmanData kalman_data;
    kalman_data.kalman_r_acc_x = kalman_state.state_est_accel_x;
    kalman_data.kalman_r_acc_y = kalman_state.state_est_accel_y;
    kalman_data.kalman_r_acc_z = kalman_state.state_est_accel_z;
    kalman_data.kalman_r_vel_x = kalman_state.state_est_vel_x;
    kalman_data.kalman_r_vel_y = kalman_state.state_est_vel_y;
    kalman_data.kalman_r_vel_z = kalman_state.state_est_vel_z;
    kalman_data.kalman_r_pos_x = kalman_state.state_est_pos_x;
    kalman_data.kalman_r_pos_y = kalman_state.state_est_pos_y;
    kalman_data.kalman_r_pos_z = kalman_state.state_est_pos_z;

    dataLogger.pushKalmanFifo(kalman_data);
}

KalmanState KalmanFilter::getState() const { return kalman_state; }

void KalmanFilter::setState(KalmanState state) { kalman_state = state; }

void KalmanFilter::updateApogee(float estimate) { kalman_apo = estimate; }

KalmanFilter kalmanFilter;
