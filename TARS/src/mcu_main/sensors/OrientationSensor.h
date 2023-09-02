#pragma once

#include "mcu_main/Rt.h"
#include <cmath>

#include "common/packet.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/debug.h"
#include "mcu_main/error.h"
#include "mcu_main/pins.h"
#include "mcu_main/hilsim/HILSIMPacket.pb.h"

#ifndef ENABLE_SILSIM_MODE
#include <Wire.h>
#include "Adafruit_BNO08x.h"
#include <Arduino.h>
#endif

/**
* 
* @class OrientationSensor
* 
* @brief This class initializes and controls the orientation sensor. One can obtain data using the functions provided in the class.
* 
* 
* 
* This class utilizes an imu that is capable of orientation. Currently the constructor can accept 0 parameters or the 
* Adafruit_BNO08x imu sensor for data collection. Using this class one can obtain temperature, pressure, gyroscope
* acceleration, and magnetometer data. One also has the choice to receive the current orientation in Euler angles or
* quaternions.
*/

class OrientationSensor;
extern OrientationSensor orientation;

class OrientationSensor {
   public:
    MUTEX_DECL(mutex);    
    OrientationSensor();

    void update();
    void update(HILSIMPacket hilsim_packet);

    ErrorCode __attribute__((warn_unused_result)) init();

    Acceleration getAccelerations();
    Gyroscope getGyroscope();
    Magnetometer getMagnetometer();
    euler_t getEuler();
    float getTemp();
    float getPressure();

#ifndef ENABLE_SILSIM_MODE
    void setIMU(Adafruit_BNO08x const& imu);
    explicit OrientationSensor(Adafruit_BNO08x const& imu);
    void setReports(sh2_SensorId_t reportType, long report_interval);
#endif

   private:

    /**
    *  Converts quaternions to Euler angles using quaternion components
    */
    void quaternionToEuler(float qr, float qi, float qj, float qk,
                           bool degrees = false);
#ifndef ENABLE_SILSIM_MODE
    /**
    *  Converts quaternions to Euler angles using rotation vectors
    */
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector,
                             bool degrees = false);

    /**
    *  Converts quaternions to Euler angles using the integration gyroscope
    */
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector,
                             bool degrees = false);
    Adafruit_BNO08x _imu;
#endif
    euler_t _orientationEuler{};
    Acceleration _accelerations{};
    Gyroscope _gyro{};
    Magnetometer _magnetometer{};
    systime_t time_stamp = 0;
    float _temp = 0.0;
    float _pressure = 0.0;
};