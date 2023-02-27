#include "OrientationSensor.h"

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void OrientationSensor::setReports(sh2_SensorId_t reportType, long report_interval) {
//    Serial.println("Setting desired reports");
//    if (!_imu.enableReport(reportType, report_interval)) {
//        Serial.println("Could not enable stabilized remote vector");
//    }
}

OrientationSensor orientation;

OrientationSensor::OrientationSensor() {
//    _imu = Adafruit_BNO08x(BNO086_RESET);
//    setReports(reportType, reportIntervalUs);
}

OrientationSensor::OrientationSensor(Adafruit_BNO08x bno) {
//    _imu = bno;
//    setReports(reportType, reportIntervalUs);
}

void OrientationSensor::setIMU(Adafruit_BNO08x bno) {
//    _imu = bno;
//    setReports(reportType, reportIntervalUs);
}

void OrientationSensor::update() {
//    chSysLock();
//    chMtxLock(&mutex);
//    sh2_SensorValue_t event;
//    if (_imu.getSensorEvent(&event)) {
//        switch (event.sensorId) {
//            case SH2_ARVR_STABILIZED_RV:
//                quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
//            case SH2_GYRO_INTEGRATED_RV:
//                // faster (more noise?)
//                quaternionToEulerGI(&event.un.gyroIntegratedRV, true);
//                break;
//        }
//
//        _accelerations.ax = event.un.accelerometer.x;
//        _accelerations.ay = event.un.accelerometer.y;
//        _accelerations.az = event.un.accelerometer.z;
//
//        _gyro.gx = event.un.gyroscope.x;
//        _gyro.gy = event.un.gyroscope.y;
//        _gyro.gz = event.un.gyroscope.z;
//
//        _magnetometer.mx = event.un.magneticField.x;
//        _magnetometer.my = event.un.magneticField.y;
//        _magnetometer.mz = event.un.magneticField.z;
//
//        _temp = event.un.temperature.value;
//        _pressure = event.un.pressure.value;
//    }
//    time_stamp = chVTGetSystemTime();
//    dataLogger.pushOrientationFifo((OrientationData){_orientationEuler.yaw, _orientationEuler.pitch, _orientationEuler.roll, time_stamp});
//    chMtxUnlock(&mutex);
//    chSysUnlock();
}

void OrientationSensor::quaternionToEuler(float qr, float qi, float qj, float qk,
                            bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    _orientationEuler.yaw =
        atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    _orientationEuler.pitch =
        asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    _orientationEuler.roll =
        atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

void OrientationSensor::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector,
                              bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i,
                      rotational_vector->j, rotational_vector->k, degrees);
}

void OrientationSensor::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector,
                              bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i,
                      rotational_vector->j, rotational_vector->k, degrees);
}

float OrientationSensor::getTemp() { return _temp; }

float OrientationSensor::getPressure() { return _pressure; }

Gyroscope OrientationSensor::getGyroscope() { return _gyro; }

Magnetometer OrientationSensor::getMagnetometer() { return _magnetometer; }

Acceleration OrientationSensor::getAccelerations() { return _accelerations; }

euler_t OrientationSensor::getEuler() { return _orientationEuler; }

OrientationSensor orientationSensor;