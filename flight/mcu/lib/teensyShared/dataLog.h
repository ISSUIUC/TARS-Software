#ifndef DATALOG_H
#define DATALOG_H

#include <ChRt.h>
#include <SD.h>
#include <stdint.h>

#include "FifoBuffer.h"
#include "KX134-1211.h"       //High-G IMU Library
#include "MS5611.h"           //Barometer Library
#include "SparkFunLSM9DS1.h"  //Low-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "acShared.h"
#include "dataStructs.h"

/**
 * @brief Structure for all values collected from the low g sensor
 *
 */
struct lowGData {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    systime_t timeStamp_lowG;
};

/**
 * @brief Structure for all values collected from the high g sensor
 *
 */
struct highGData {
    float hg_ax;
    float hg_ay;
    float hg_az;
    systime_t timeStamp_highG;
};

/**
 * @brief Structure for all values collected from the gps
 *
 */
struct gpsData {
    float latitude;
    float longitude;
    float altitude;
    uint32_t siv_count;
    uint32_t fix_type;
    bool posLock;
    systime_t timeStamp_GPS;
};
struct flapData {
    float l1;
    float l2;
    systime_t timeStamp_flaps;
};

/**
 * @brief Structure for all values collected from the barometer
 *
 */
struct barometerData {
    float temperature;  // in degC
    float pressure;     // in mbar
    float altitude;     // in meter
    int32_t timeStamp_barometer;
};

/**
 * @brief Structure for all values tracked for state estimation
 *
 */
struct stateData {
    float state_q0 = 0;
    float state_q1 = 0;
    float state_q2 = 0;
    float state_q3 = 0;

    float state_x = 0;
    float state_y = 0;
    float state_z = 0;

    float state_vx = 0;
    float state_vy = 0;
    float state_vz = 0;

    float state_ax = 0;
    float state_ay = 0;
    float state_az = 0;

    float state_omegax = 0;
    float state_omegay = 0;
    float state_omegaz = 0;

    float state_latitude = 0;
    float state_longitude = 0;

    systime_t timeStamp_state = 0;
};

/**
 * @brief Structure for all values related to rocket state
 *
 */
struct rocketStateData {
    FSM_State rocketState = STATE_INIT;
    systime_t timeStamp_RS = 0;
};

/**
 * @brief A struct to hold all of the data that could come from any of the
 * sensors
 *
 */
struct sensorDataStruct_t {
    // data for lowGimu
    bool has_lowG_data;
    lowGData lowG_data;

    // data for highGimu accel data (hg_x, hg_y, hg_z)
    bool has_highG_data;
    highGData highG_data;

    // GPS DATA
    bool has_gps_data;
    gpsData gps_data;

    // Barometer data (temp and pres)
    bool has_barometer_data;
    barometerData barometer_data;

    // State variables
    bool has_state_data;
    stateData state_data;

    // Rocket State
    bool has_rocketState_data;
    rocketStateData rocketState_data;

    // Flap state
    bool has_flap_data;
    flapData flap_data;
};

/**
 * @brief An enum to list all potential sensors.
 *
 */
enum sensors { LOWG_IMU, HIGHG_IMU, BAROMETER, GPS };

#define FIFO_SIZE 1000
/**
 * @brief A struct to hold all info for ring buffers and mutexes used for data.
 *
 */
struct datalogger_THD {
    FifoBuffer<lowGData, FIFO_SIZE> lowGFifo{};
    FifoBuffer<highGData, FIFO_SIZE> highGFifo{};
    FifoBuffer<gpsData, FIFO_SIZE> gpsFifo{};
    FifoBuffer<stateData, FIFO_SIZE> stateFifo{};
    FifoBuffer<rocketStateData, FIFO_SIZE> rocketStateFifo{};
    FifoBuffer<barometerData, FIFO_SIZE> barometerFifo{};
    FifoBuffer<flapData, FIFO_SIZE> flapFifo{};

    MUTEX_DECL(dataMutex_lowG);
    MUTEX_DECL(dataMutex_highG);
    MUTEX_DECL(dataMutex_GPS);
    MUTEX_DECL(dataMutex_barometer);
    MUTEX_DECL(dataMutex_state);
    MUTEX_DECL(dataMutex_RS);

    sensorDataStruct_t current_data;

    File dataFile;
};

// TODO: Re-think this struct
struct pointers {
    LSM9DS1* lowGimuPointer;
    KX134* highGimuPointer;
    MS5611* barometerPointer;
    SFE_UBLOX_GNSS* GPSPointer;

    sensorDataStruct_t* sensorDataPointer;

    datalogger_THD dataloggerTHDVarsPointer;
};

void dataLoggerTickFunction(pointers*);

char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, sensorDataStruct_t* data);

#endif
