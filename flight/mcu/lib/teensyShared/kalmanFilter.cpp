
#include "kalmanFilter.h"
#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

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
    current_state_ = &pointer_struct->sensorDataPointer->rocketState_data.rocketState;   
}

void KalmanFilter::kfTickFunction() {
    if(*current_state_ > STATE_IDLE){
        priori();
        update();
    }
}

void KalmanFilter::Initialize() {
    float sum = 0;
    for(int i = 0; i < 30; i++){
        chMtxLock(dataMutex_barometer_);
        // std::cout<<baro_data_ptr_->altitude<<std::endl;
        sum += *b_alt;
        chMtxUnlock(dataMutex_barometer_);
        chThdSleepMilliseconds(100);
    }

    // set x_k
    x_k(0,0) = sum / 30;
    // x_k(0,0) = 1401;
    x_k(1,0) = 0;
    x_k(2,0) = 0;
    
    // set F
    F_mat(0, 1) = s_dt;
    F_mat(0, 2) = (s_dt*s_dt) / 2;
    F_mat(1, 2) = s_dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;
    F_mat(2, 2) = 1;

    // set H
    H(0,0) = 1;
    H(1,2) = 1;

    // set P_k
    P_k(0,0) = 0;
    P_k(0,1) = 0;
    P_k(0,2) = 0;
    P_k(1,1) = 0;
    P_k(2,2) = 0;
    P_k(1,2) = 0;
    P_k(2,1) = P_k(1,2);
    P_k(1,0) = P_k(0,1);
    P_k(2,0) = P_k(0,2);
    P_k(1,0) = P_k(0,1);

    // set Q
    Q(0,0) = pow(s_dt,5) / 20;
    Q(0,1) = (pow(s_dt,4) / 8 * 80);
    Q(0,2) = pow(s_dt,3) / 6;
    Q(1,1) = pow(s_dt,3) / 8;
    Q(1,2) = pow(s_dt,2) / 2;
    Q(2,2) = s_dt;
    Q(1,0) = Q(0,1);
    Q(2,0) = Q(0,2);
    Q(2,1) = Q(1,2);

    // float scale_fact = 75.19;
    // float scale_fact = 14.25;
    float scale_fact = .00999;
    // float scale_fact = 13;
    Q = Q * scale_fact;

    // set R
    R(0,0) = 5.;
    R(1,1) = .0002;
    // R(0,0) = 2.;
    // R(1,1) = .01;

    // set B
    B(2,0) = -1;
}

void KalmanFilter::Initialize(float pos_f, float vel_f) {
    // set x_k
    x_k(0,0) = pos_f;
    x_k(1,0) = vel_f;
    
    // set F
    F_mat(0, 1) = s_dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;

    // set H
    H(0,0) = 1;

    // set R
    R(0,0) = 12;

    // set B
    B(2,0) = -1;
}

void KalmanFilter::priori() {
    // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B or u is = 0
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void KalmanFilter::update() {
 
    // Update Kalman Gain
    temp = (((H * P_priori * H.transpose()) + R)).inverse();
    K = (P_priori * H.transpose()) * temp;

    // Sensor Measurements
    chMtxLock(mutex_highG_);
    y_k(1,0) = (*gz_H) * 9.81;
    // Serial.println("HIGH G ACCEL Z: ");
    // Serial.println((*gz_H)*9.81);
    chMtxUnlock(mutex_highG_);

    chMtxLock(dataMutex_barometer_);
    y_k(0,0) = *b_alt;
    // Serial.println(y_k(0,0));
    // std::cout<< y_k(0,0) <<std::endl;
    chMtxUnlock(dataMutex_barometer_);
    
    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K*H) * P_priori;

    chMtxLock(dataMutex_state_);
    stateData_->state_x = x_k(0,0);
    stateData_->state_vx = x_k(1,0);
    stateData_->state_ax = x_k(2,0);
    stateData_->timeStamp_state = chVTGetSystemTime();
    chMtxUnlock(dataMutex_state_);
    data_logger_->pushStateFifo(stateData_);
}
