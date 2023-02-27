#pragma once

#include <ChRt.h>
#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_BNO08x.h"
#include "mcu_main/pins.h"
#include "mcu_main/dataLog.h"
#include "common/packet.h"



class OrientationSensor {
   public:
    MUTEX_DECL(mutex);
    
    OrientationSensor();
    OrientationSensor(Adafruit_BNO08x const& imu);

    void update();
    void init();

    void setIMU(Adafruit_BNO08x const& imu);
    Acceleration getAccelerations();
    Gyroscope getGyroscope();
    Magnetometer getMagnetometer();
    euler_t getEuler();
    float getTemp();
    float getPressure();
    void setReports(sh2_SensorId_t reportType, long report_interval);

   private:
    void quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false);

    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees = false);

    Adafruit_BNO08x _imu;
    euler_t _orientationEuler{};
    Acceleration _accelerations{};
    Gyroscope _gyro{};
    Magnetometer _magnetometer{};
    systime_t time_stamp = 0;
    float _temp = 0.0;
    float _pressure = 0.0;
};