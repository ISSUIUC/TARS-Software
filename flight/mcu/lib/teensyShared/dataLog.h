#ifndef DATALOG_H
#define DATALOG_H

#include <ChRt.h>
#include <SD.h>

#include "FifoBuffer.h"
#include "MS5611.h"                //Barometer Library
#include "SparkFunLSM9DS1.h"       //Low-G IMU Library
#include "SparkFun_Qwiic_KX13X.h"  //High-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "VoltageSensor.h"
#include "RocketFSMBase.h"
#include "HistoryBuffer.h"

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
    FifoBuffer<KalmanData, FIFO_SIZE> kalmanFifo{};
    FifoBuffer<rocketStateData<4>, FIFO_SIZE> rocketStateFifo{};
    FifoBuffer<FlapData, FIFO_SIZE> flapFifo{};
    FifoBuffer<VoltageData, FIFO_SIZE> voltageFifo{};
    FifoBuffer<BarometerData, FIFO_SIZE> barometerFifo{};

   public:
    HistoryBuffer<50> altitude_history_50 = HistoryBuffer<50>();
    HistoryBuffer<50> IMU_acceleration_history_50 = HistoryBuffer<50>();

    HistoryBuffer<6> altitude_history_6 = HistoryBuffer<6>();
    HistoryBuffer<6> IMU_acceleration_history_6 = HistoryBuffer<6>();

    HistoryBuffer<6> gnc_altitude_history_6 = HistoryBuffer<6>();
    HistoryBuffer<6> gnc_IMU_acceleration_history_6 = HistoryBuffer<6>();

    File dataFile;

    void init();

    bool pushLowGFifo(LowGData const& lowG_Data);
    bool popLowGFifo(LowGData* lowG_Data);

    bool pushHighGFifo(HighGData const& highG_Data);
    bool popHighGFifo(HighGData* highG_Data);

    bool pushGpsFifo(GpsData const& gps_Data);
    bool popGpsFifo(GpsData* gps_Data);

    bool pushKalmanFifo(KalmanData const& state_data);
    bool popKalmanFifo(KalmanData* state_data);

    bool pushRocketStateFifo(rocketStateData<4> const& rocket_data);
    bool popRocketStateFifo(rocketStateData<4>* rocket_data);

    bool pushBarometerFifo(BarometerData const& barometer_data);
    bool popBarometerFifo(BarometerData* barometer_data);

    bool pushFlapsFifo(FlapData const& flap_data);
    bool popFlapsFifo(FlapData* flap_data);

    bool pushVoltageFifo(VoltageData const& voltage_data);
    bool popVoltageFifo(VoltageData* voltage_data);
};

extern DataLogBuffer dataLogger;

void dataLoggerTickFunction();

char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, sensorDataStruct_t* data);

#endif
