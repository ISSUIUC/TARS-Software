#include "OrientationSensor.h"

#include <cmath>

#include "mcu_main/debug.h"

OrientationSensor orientation;

#ifdef ENABLE_SILSIM_MODE
#include "mcu_main/emulation.h"
#elif
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
#ifdef ENABLE_ORIENTATION
    Serial.println("Setting desired reports");
    if (!_imu.enableReport(reportType, report_interval)) {
        Serial.println("Could not enable stabilized remote vector");
    }
#endif
}
#endif

#if defined(ENABLE_ORIENTATION) && !defined(ENABLE_SILSIM_MODE)
OrientationSensor::OrientationSensor() : _imu(Adafruit_BNO08x(BNO086_RESET)) {}
#else
OrientationSensor::OrientationSensor() = default;
#endif

#ifndef ENABLE_SILSIM_MODE
OrientationSensor::OrientationSensor(Adafruit_BNO08x const& bno) {
#ifdef ENABLE_ORIENTATION
    _imu = bno;
    setReports(reportType, reportIntervalUs);
#endif
}

void OrientationSensor::setIMU(Adafruit_BNO08x const& bno) {
#ifdef ENABLE_ORIENTATION
    _imu = bno;
    setReports(reportType, reportIntervalUs);
#endif
}
#endif

void OrientationSensor::update() {
#ifdef ENABLE_ORIENTATION
    chSysLock();
    chMtxLock(&mutex);
#ifdef ENABLE_SILSIM_MODE
    auto data = emulatedKX->get_data();
    _accelerations.ax = data.x();
    _accelerations.ay = data.y();
    _accelerations.az = data.z();
    _gyro.gx = emulatedGyro->get_data().x();
    _gyro.gy = emulatedGyro->get_data().y();
    _gyro.gz = emulatedGyro->get_data().z();
    _pressure = emulatedMS->get_data();
    _temp = emulatedThermometer->get_data();
    _magnetometer.mx = emulatedMagnetometer->get_data().x();
    _magnetometer.my = emulatedMagnetometer->get_data().y();
    _magnetometer.mz = emulatedMagnetometer->get_data().z();
#elif
    sh2_SensorValue_t event;
    if (_imu.getSensorEvent(&event)) {
        switch (event.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
                quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
            case SH2_GYRO_INTEGRATED_RV:
                // faster (more noise?)
                quaternionToEulerGI(&event.un.gyroIntegratedRV, true);
                break;
        }

        _accelerations.ax = event.un.accelerometer.x;
        _accelerations.ay = event.un.accelerometer.y;
        _accelerations.az = event.un.accelerometer.z;

        _gyro.gx = event.un.gyroscope.x;
        _gyro.gy = event.un.gyroscope.y;
        _gyro.gz = event.un.gyroscope.z;

        _magnetometer.mx = event.un.magneticField.x;
        _magnetometer.my = event.un.magneticField.y;
        _magnetometer.mz = event.un.magneticField.z;

        _temp = event.un.temperature.value;
        _pressure = event.un.pressure.value;
    }
#endif
    time_stamp = chVTGetSystemTime();
    dataLogger.pushOrientationFifo(
        (OrientationData){_accelerations, _gyro, _magnetometer, _orientationEuler, time_stamp});
    chMtxUnlock(&mutex);
    chSysUnlock();
#endif
}

void OrientationSensor::update(HILSIMPacket hilsim_packet) {
#ifdef ENABLE_ORIENTATION
    chSysLock();
    chMtxLock(&mutex);
    _orientationEuler = (euler_t) {hilsim_packet.ornt_roll, hilsim_packet.ornt_pitch, hilsim_packet.ornt_yaw};
    dataLogger.pushOrientationFifo(
        (OrientationData){_accelerations, _gyro, _magnetometer, _orientationEuler, chVTGetSystemTime()});
    chMtxUnlock(&mutex);
    chSysUnlock();
#endif
}

#define sq(v) (std::pow(v, 2))

void OrientationSensor::quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    _orientationEuler.roll = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    _orientationEuler.yaw = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    _orientationEuler.pitch = -1 * atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

#ifndef ENABLE_SILSIM_MODE
void OrientationSensor::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                      degrees);
}

void OrientationSensor::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                      degrees);
}
#endif

float OrientationSensor::getTemp() { return _temp; }

float OrientationSensor::getPressure() { return _pressure; }

Gyroscope OrientationSensor::getGyroscope() { return _gyro; }

Magnetometer OrientationSensor::getMagnetometer() { return _magnetometer; }

Acceleration OrientationSensor::getAccelerations() { return _accelerations; }

euler_t OrientationSensor::getEuler() { return _orientationEuler; }

ErrorCode OrientationSensor::init() {
#ifndef ENABLE_SILSIM_MODE
    if (!_imu.begin_SPI(BNO086_CS, BNO086_INT)) {
        return ErrorCode::CANNOT_CONNECT_BNO;
    }
    setReports(reportType, reportIntervalUs);
    Serial.println("Setting desired reports");
    if (!_imu.enableReport(reportType, reportIntervalUs)) {
        Serial.println("Could not enable stabilized remote vector");
        return ErrorCode::CANNOT_INIT_BNO;
    }
#endif
    return ErrorCode::NO_ERROR;
}