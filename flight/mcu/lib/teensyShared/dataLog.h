#ifndef DATALOG_H
#define DATALOG_H

#include <ChRt.h>
#include <SD.h>
#include <stdint.h>

#include "FifoBuffer.h"
#include "MS5611.h"                //Barometer Library
#include "SparkFunLSM9DS1.h"       //Low-G IMU Library
#include "SparkFun_Qwiic_KX13X.h"  //High-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "VoltageSensor.h"
#include "rocketFSM.h"
// #include "acShared.h"
// #include "dataStructs.h"

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
    float mx;
    float my;
    float mz;
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
 * @brief Structure for all values tracked for state estimation
 *
 */
struct stateData {
    float state_x = 0;
    float state_vx = 0;
    float state_ax = 0;
    float state_apo = 0;
    // float state_q0 = 0;

    systime_t timeStamp_state = 0;
};

/**
 * @brief A struct to hold all of the data that could come from any of the
 * sensors
 *
 */
struct sensorDataStruct_t {
    // data for lowGimu
    bool has_lowG_data;
    LowGData lowG_data;

    // data for highGimu accel data (hg_x, hg_y, hg_z)
    bool has_highG_data;
    HighGData highG_data;

    // GPS DATA
    bool has_gps_data;
    GpsData gps_data;

    // Barometer data (temp and pres)
    bool has_barometer_data;
    BarometerData barometer_data;

    // State variables
    bool has_state_data;
    stateData state_data;

    // Rocket State
    bool has_rocketState_data;
    rocketStateData rocketState_data;

    // Flap state
    bool has_flap_data;
    FlapData flap_data;

    // Voltage state
    bool has_voltage_data;
    VoltageData voltage_data;
};

/**
 * @brief An enum to list all potential sensors.
 *
 */
enum sensors { LOWG_IMU, HIGHG_IMU, BAROMETER, GPS };

#define FIFO_SIZE 1000
/**
 * @brief A class to hold all info for ring buffers and mutexes used for data.
 *
 */

class DataLogBuffer {
   private:
    FifoBuffer<LowGData, FIFO_SIZE> lowGFifo{};
    FifoBuffer<HighGData, FIFO_SIZE> highGFifo{};
    FifoBuffer<GpsData, FIFO_SIZE> gpsFifo{};
    FifoBuffer<stateData, FIFO_SIZE> stateFifo{};
    FifoBuffer<rocketStateData, FIFO_SIZE> rocketStateFifo{};
    FifoBuffer<FlapData, FIFO_SIZE> flapFifo{};
    FifoBuffer<VoltageData, FIFO_SIZE> voltageFifo{};
    FifoBuffer<BarometerData, FIFO_SIZE> barometerFifo{};

   public:
    MUTEX_DECL(dataMutex_lowG);
    MUTEX_DECL(dataMutex_highG);
    MUTEX_DECL(dataMutex_GPS);
    MUTEX_DECL(dataMutex_barometer);
    MUTEX_DECL(dataMutex_flaps);
    MUTEX_DECL(dataMutex_rocket_state);
    MUTEX_DECL(dataMutex_voltage);
    MUTEX_DECL(dataMutex_state);

    sensorDataStruct_t current_data;

    File dataFile;

    bool pushLowGFifo(LowGData* lowG_Data);
    bool popLowGFifo(LowGData* lowG_Data);

    bool pushHighGFifo(HighGData* highG_Data);
    bool popHighGFifo(HighGData* highG_Data);

    bool pushGpsFifo(GpsData* gps_Data);
    bool popGpsFifo(GpsData* gps_Data);

    bool pushStateFifo(stateData* state_data);
    bool popStateFifo(stateData* state_data);

    bool pushRocketStateFifo(rocketStateData* rocket_data);
    bool popRocketStateFifo(rocketStateData* rocket_data);

    bool pushBarometerFifo(BarometerData* barometer_data);
    bool popBarometerFifo(BarometerData* barometer_data);

    bool pushFlapsFifo(FlapData* flap_data);
    bool popFlapsFifo(FlapData* flap_data);

    bool pushVoltageFifo(VoltageData* voltage_data);
    bool popVoltageFifo(VoltageData* voltage_data);
};

// TODO: Re-think this struct
struct pointers {
    LSM9DS1* lowGimuPointer;
    QwiicKX134* highGimuPointer;
    MS5611* barometerPointer;
    SFE_UBLOX_GNSS* GPSPointer;

    sensorDataStruct_t* sensorDataPointer;

    DataLogBuffer dataloggerTHDVarsPointer;
    bool abort;
};

void dataLoggerTickFunction(pointers*);

char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, sensorDataStruct_t* data);

#endif
