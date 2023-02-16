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
    Serial.println("Setting desired reports");
    if (!_imu->enableReport(reportType, report_interval)) {
        Serial.println("Could not enable stabilized remote vector");
    }
}

OrientationSensor::OrientationSensor(Adafruit_BNO08x* bno) {
    _imu = bno;
    setReports(reportType, reportIntervalUs);
}

void OrientationSensor::setIMU(Adafruit_BNO08x* bno) {
    _imu = bno;
    setReports(reportType, reportIntervalUs);
}

void OrientationSensor::readData() {
    sh2_SensorValue_t event;
    if (_imu->getSensorEvent(&event)) {
        switch (event.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
                quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
            case SH2_GYRO_INTEGRATED_RV:
                // faster (more noise?)
                quaternionToEulerGI(&event.un.gyroIntegratedRV, true);
                break;
        }

        _accelerations.x = event.un.accelerometer.x;
        _accelerations.y = event.un.accelerometer.y;
        _accelerations.z = event.un.accelerometer.z;

        _gyro.x = event.un.gyroscope.x;
        _gyro.y = event.un.gyroscope.y;
        _gyro.z = event.un.gyroscope.z;

        _magnetometer.x = event.un.magneticField.x;
        _magnetometer.y = event.un.magneticField.y;
        _magnetometer.z = event.un.magneticField.z;

        _temp = event.un.temperature.value;
        _pressure = event.un.pressure.value;
    }
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

Accelerations OrientationSensor::getAccelerations() { return _accelerations; }

euler_t OrientationSensor::getEuler() { return _orientationEuler; }
