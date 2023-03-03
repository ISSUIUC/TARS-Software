#pragma once

#include "ChRt.h"

struct Acceleration {
    float ax;
    float ay;
    float az;
};

struct Gyroscope {
    float gx;
    float gy;
    float gz;
};

struct Magnetometer {
    float mx;
    float my;
    float mz;
};

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

/**
 * @brief Labels for each FSM state
 */
enum class FSM_State {
    STATE_UNKNOWN,
    STATE_INIT,
    STATE_IDLE,
    STATE_LAUNCH_DETECT,
    STATE_BOOST,
    STATE_BURNOUT_DETECT,
    STATE_COAST_PREGNC,
    STATE_COAST_GNC,
    STATE_APOGEE_DETECT,
    STATE_APOGEE,
    STATE_SEPARATION,
    STATE_DROGUE_DETECT,
    STATE_DROGUE,
    STATE_MAIN_DETECT,
    STATE_MAIN,
    STATE_LANDED_DETECT,
    STATE_LANDED,
    STATE_ABORT
};

/**
 * @brief Structure for all values related to rocket state
 *
 */
template <size_t count>
struct rocketStateData {
    FSM_State rocketStates[count];
    systime_t timestamp = 0;

    rocketStateData() : rocketStates() {
        for (size_t i = 0; i < count; i++) {
            rocketStates[i] = FSM_State::STATE_INIT;
        }
    }
};

struct VoltageData {
    float v_battery;
    systime_t timestamp;
};

/**
 * @brief Structure for all values collected from the low g sensor
 *
 */
struct LowGData {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    //    float mx;
    //    float my;
    //    float mz;
    systime_t timeStamp_lowG;
};

/**
 * @brief Structure for all values collected from the high g sensor
 *
 */
struct HighGData {
    float hg_ax;
    float hg_ay;
    float hg_az;
    systime_t timeStamp_highG;
};

/**
 * @brief Structure for all values collected from the gps
 *
 */
struct GpsData {
    float latitude;
    float longitude;
    float altitude;
    uint32_t siv_count;
    uint32_t fix_type;
    bool posLock;
    systime_t timeStamp_GPS;
};

/**
 * @brief Structure for values relevant to active control flaps
 *
 */
struct FlapData {
    float extension;
    systime_t timeStamp_flaps;
};

/**
 * @brief Structure for all values collected from the barometer
 *
 */
struct BarometerData {
    float temperature;  // in degC
    float pressure;     // in mbar
    float altitude;     // in meter
    systime_t timeStamp_barometer;
};

/**
 * @brief Structure for all values tracked for kalman state estimation
 *
 */
struct KalmanData {
    float kalman_pos_x = 0;
    float kalman_vel_x = 0;
    float kalman_acc_x = 0;
    float kalman_pos_y = 0;
    float kalman_vel_y = 0;
    float kalman_acc_y = 0;
    float kalman_pos_z = 0;
    float kalman_vel_z = 0;
    float kalman_acc_z = 0;
    float kalman_apo = 0;

    systime_t timeStamp_state = 0;
};

struct OrientationData {
    Acceleration accel{};
    Gyroscope gyro{};
    Magnetometer magnet{};
    euler_t angle{};
    systime_t timeStamp_orientation = 0;
};

struct GasData {
    float temp = 0.0;
    float humidity = 0.0;
    uint32_t pressure = 0;
    uint32_t resistance = 0;
    systime_t timestamp = 0;
};

struct MagnetometerData {
    Magnetometer magnetometer{};
    systime_t timestamp = 0;
};

/**
 * @brief A struct to hold all of the data that could come from any of the
 * sensors
 *
 */
struct sensorDataStruct_t {
    // data for lowGimu
    bool has_lowG_data = false;
    LowGData lowG_data{};

    // data for highGimu accel data (hg_x, hg_y, hg_z)
    bool has_highG_data = false;
    HighGData highG_data{};

    // GPS DATA
    bool has_gps_data = false;
    GpsData gps_data{};

    // Barometer data (temp and pres)
    bool has_barometer_data = false;
    BarometerData barometer_data{};

    // State variables
    bool has_kalman_data = false;
    KalmanData kalman_data{};

    // Rocket State
    bool has_rocketState_data = false;
    rocketStateData<4> rocketState_data{};

    // Flap state
    bool has_flap_data = false;
    FlapData flap_data{};

    // Voltage state
    bool has_voltage_data = false;
    VoltageData voltage_data{};

    bool has_orientation_data = false;
    OrientationData orientation_data{};

    bool has_magnetometer_data = false;
    MagnetometerData magnetometer_data{};

    bool has_gas_data = false;
    GasData gas_data{};

    bool hasData() const {
        return has_gps_data || has_highG_data || has_lowG_data || has_rocketState_data || has_kalman_data ||
               has_barometer_data || has_flap_data || has_voltage_data || has_orientation_data ||
               has_magnetometer_data || has_gas_data;
    }
};
