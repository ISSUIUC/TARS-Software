#pragma once

#include "Adafruit_BNO08x.h"
#include <Arduino.h>
#include <Wire.h>
#include "mcu_main/pins.h"
#include <ChRt.h>

#include <cmath>

class OrientationSensor;
extern OrientationSensor orientationSensor;

struct BNO_Accelerations {
    float x;
    float y;
    float z;
};

struct BNO_Gyroscope {
    // rad/s
    float x;
    float y;
    float z;
};

struct BNO_Magnetometer {
    // uT
    float x;
    float y;
    float z;
};

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

class OrientationSensor {
   public:
    MUTEX_DECL(mutex);
    
    OrientationSensor();
    OrientationSensor(Adafruit_BNO08x* imu);
    void update();
    void setIMU(Adafruit_BNO08x* imu);
    BNO_Accelerations getAccelerations();
    BNO_Gyroscope getGyroscope();
    BNO_Magnetometer getMagnetometer();
    euler_t getEuler();
    float getTemp();
    float getPressure();
    void setReports(sh2_SensorId_t reportType, long report_interval);

   private:
    void quaternionToEuler(float qr, float qi, float qj, float qk,
                           bool degrees = false);

    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector,
                             bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector,
                             bool degrees = false);

    Adafruit_BNO08x* _imu;
    euler_t _orientationEuler;
    BNO_Accelerations _accelerations;
    BNO_Gyroscope _gyro;
    BNO_Magnetometer _magnetometer;
    float _temp;
    float _pressure;
};