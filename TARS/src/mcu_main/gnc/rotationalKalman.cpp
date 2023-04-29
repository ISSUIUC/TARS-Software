#include "mcu_main/gnc/rotationalKalman.h"

#include <cmath>

#include "mcu_main/finite-state-machines/rocketFSM.h"

#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

RotationalKalmanFilter::RotationalKalmanFilter() : KalmanFilter() {}

void RotationalKalmanFilter::SetQ(float dt, float sd) {
    for (int i = 0; i < 3; i ++) {
        Q(3*i, 3*i) = pow(dt, 5) / 20;
        Q(3*i, 3*i+1) = pow(dt, 4) / 8;
        Q(3*i, 3*i+2) = pow(dt, 3) / 6;
        Q(3*i+1, 3*i+1) = pow(dt, 3) / 8;
        Q(3*i+1, 3*i+2) = pow(dt, 2) / 2;
        Q(3*i+2, 3*i+2) = dt;
        Q(3*i+1, 3*i) = Q(3*i, 3*i+1);
        Q(3*i+2, 3*i) = Q(3*i, 3*i+2);
        Q(3*i+2, 3*i+1) = Q(3*i+1, 3*i+2);
    }

    Q *= sd;
}

void RotationalKalmanFilter::SetF(float dt) {
    for (int i = 0; i < 3; i++) {
        F_mat(3 * i, 3 * i + 1) = dt;
        F_mat(3 * i, 3 * i + 2) = (dt * dt) / 2;
        F_mat(3 * i + 1, 3 * i + 2) = dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }
}

void RotationalKalmanFilter::Initialize() {

    for (int i = 0; i < 3; i++) {
        // set F
        F_mat(3 * i, 3 * i + 1) = s_dt;
        F_mat(3 * i, 3 * i + 1) = (s_dt * s_dt) / 2;
        F_mat(3 * i + 1, 3 * i + 2) = s_dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;

        // set Q
        Q(3 * i, 3 * i) = pow(s_dt, 5) / 20;
        Q(3 * i, 3 * i + 1) = pow(s_dt, 4) / 8 * 80;
        Q(3 * i, 3 * i + 2) = pow(s_dt, 3) / 6;
        Q(3 * i + 1, 3 * i + 1) = pow(s_dt, 3) / 8;
        Q(3 * i + 1, 3 * i + 2) = pow(s_dt, 2) / 2;
        Q(3 * i + 2, 3 * i + 2) = s_dt;
        Q(3 * i + 1, 3 * i) = Q(3 * i, 3 * i + 1);
        Q(3 * i + 2, 3 * i) = Q(3 * i, 3 * i + 2);
        Q(3 * i + 2, 3 * i + 1) = Q(3 * i + 1, 3 * i + 2);
    }

    H(0, 0) = 1;
    H(0, 1) = 1;
    H(1, 2) = 1;
    H(1, 3) = 1;
    H(2, 4) = 1;
    H(2, 5) = 1;

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

    float scale_fact = 13.0;
    Q = Q * scale_fact;

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

    for (int i = 0; i < 30; i++) {
        chMtxLock(&lowG.mutex);
        x_gyro_avg += lowG.getGyroscope().gx;
        y_gyro_avg += lowG.getGyroscope().gy;
        z_gyro_avg += lowG.getGyroscope().gz;
        chThdSleepMilliseconds(10);
        chMtxUnlock(&lowG.mutex);
    }

    x_gyro_avg /= 30;
    y_gyro_avg /= 30;
    z_gyro_avg /= 30;

    // x_k(1, 0) = x_gyro_avg / 30;
    // x_k(4, 0) = y_gyro_avg / 30;
    // x_k(7, 0) = z_gyro_avg / 30;


    R(0, 0) = 1.9;
    R(1, 1) = 1.9;
    R(2, 2) = 1.9;
    R(3, 3) = 1.9;
    R(4, 4) = 1.9;
    R(5, 5) = 1.9;

    B(2, 0) = -1;
}

void RotationalKalmanFilter::priori() {
    // Predict state
    x_priori = F_mat * x_k;

    // Predict covariance
    P_priori = F_mat * P_k * F_mat.transpose() + rqf*Q;
}

void RotationalKalmanFilter::update() {
    Eigen::Matrix<float, 6, 6> temp = Eigen::Matrix<float, 6, 6>::Zero();
    temp = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * temp;

    // TODO These mutex locks are almost certainly not necessary
    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> gyro = Eigen::Matrix<float, 3, 1>::Zero();
    chMtxLock(&lowG.mutex);
    gyro(0, 0) = lowG.getGyroscope().gx - x_gyro_avg;
    gyro(1, 0) = lowG.getGyroscope().gy - y_gyro_avg;
    gyro(2, 0) = lowG.getGyroscope().gz - z_gyro_avg;
    chMtxUnlock(&lowG.mutex);

    x_accum += gyro(0, 0) * s_dt;
    y_accum += gyro(1, 0) * s_dt;
    z_accum += gyro(2, 0) * s_dt;

    Serial.print("Accum: ");
    Serial.print(x_accum);
    Serial.print(", ");
    Serial.print(y_accum);
    Serial.print(", ");
    Serial.println(z_accum);

    // Serial.print("Gyro: ");
    // Serial.print(gyro(0, 0));
    // Serial.print(", ");
    // Serial.print(gyro(1, 0));
    // Serial.print(", ");
    // Serial.println(gyro(2, 0));

    euler_t angles;
    angles.roll = x_k(0);
    angles.pitch = x_k(1);
    angles.yaw = x_k(2);

    Eigen::Matrix<float, 3, 1> euler = BodyToGlobal(angles, gyro);
    y_k(0, 0) = euler(0);
    y_k(2, 0) = euler(1);
    y_k(4, 0) = euler(2);

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    // Serial.print("Angular Pos: ");
    // Serial.print(x_k(0, 0));
    // Serial.print(", ");
    // Serial.print(x_k(3, 0));
    // Serial.print(", ");
    // Serial.println(x_k(6, 0));
    // Serial.print("Angular Vel: ");
    // Serial.print(x_k(1, 0));
    // Serial.print(", ");
    // Serial.print(x_k(4, 0));
    // Serial.print(", ");
    // Serial.println(x_k(7, 0));
    // Serial.print("Angular Accel: ");
    // Serial.print(x_k(2, 0));
    // Serial.print(", ");
    // Serial.print(x_k(5, 0));
    // Serial.print(", ");
    // Serial.println(x_k(8, 0));

    chMtxLock(&mutex);
    rotational_kalman_state.state_est_r_pos_x = x_k(0, 0);
    rotational_kalman_state.state_est_r_vel_x = x_k(1, 0);
    rotational_kalman_state.state_est_r_accel_x = x_k(2, 0);
    rotational_kalman_state.state_est_r_pos_y = x_k(3, 0);
    rotational_kalman_state.state_est_r_vel_y = x_k(4, 0);
    rotational_kalman_state.state_est_r_accel_y = x_k(5, 0);
    rotational_kalman_state.state_est_r_pos_z = x_k(6, 0);
    rotational_kalman_state.state_est_r_vel_z = x_k(7, 0);
    rotational_kalman_state.state_est_r_accel_z = x_k(8, 0);
    chMtxUnlock(&mutex);

}

KalmanState RotationalKalmanFilter::getState() const {
    return rotational_kalman_state;
}

    void RotationalKalmanFilter::setState(KalmanState state) {
    rotational_kalman_state = state;
}

RotationalKalmanFilter rotationalKalmanFilter = RotationalKalmanFilter();
