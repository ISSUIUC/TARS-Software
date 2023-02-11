#ifndef DATALOG_H
#define DATALOG_H

#include <ChRt.h>
#include <SD.h>

#include "FifoBuffer.h"
//#include "HistoryBuffer.h"
#include "MS5611.h"                //Barometer Library
#include "SparkFunLSM9DS1.h"       //Low-G IMU Library
#include "SparkFun_Qwiic_KX13X.h"  //High-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "VoltageSensor.h"
#include "RocketFSMBase.h"
//#include "HistoryBuffer.h"

#define FIFO_SIZE 1000

class DataLogBuffer;

class DataLogView {
public:
    explicit DataLogView(DataLogBuffer& buffer);

    sensorDataStruct_t read();
private:
    FifoView<LowGData, FIFO_SIZE> lowGView;
    FifoView<HighGData, FIFO_SIZE> highGView;
    FifoView<GpsData, FIFO_SIZE> gpsView;
    FifoView<KalmanData, FIFO_SIZE> kalmanView;
    FifoView<rocketStateData<4>, FIFO_SIZE> rocketStateView;
    FifoView<FlapData, FIFO_SIZE> flapView;
    FifoView<VoltageData, FIFO_SIZE> voltageView;
    FifoView<BarometerData, FIFO_SIZE> barometerView;
};

/**
 * @brief A class to hold all info for ring buffers and mutexes used for data.
 *
 */
class DataLogBuffer {
public:
    FifoBuffer<LowGData, FIFO_SIZE> lowGFifo{};
    FifoBuffer<HighGData, FIFO_SIZE> highGFifo{};
    FifoBuffer<GpsData, FIFO_SIZE> gpsFifo{};
    FifoBuffer<KalmanData, FIFO_SIZE> kalmanFifo{};
    FifoBuffer<rocketStateData<4>, FIFO_SIZE> rocketStateFifo{};
    FifoBuffer<FlapData, FIFO_SIZE> flapFifo{};
    FifoBuffer<VoltageData, FIFO_SIZE> voltageFifo{};
    FifoBuffer<BarometerData, FIFO_SIZE> barometerFifo{};

public:
    friend class DataLogView;

    void pushLowGFifo(LowGData const& lowG_Data);
    void pushHighGFifo(HighGData const& highG_Data);
    void pushGpsFifo(GpsData const& gps_Data);
    void pushKalmanFifo(KalmanData const& state_data);
    void pushRocketStateFifo(rocketStateData<4> const& rocket_data);
    void pushBarometerFifo(BarometerData const& barometer_data);
    void pushFlapsFifo(FlapData const& flap_data);
    void pushVoltageFifo(VoltageData const& voltage_data);
};

#undef FIFO_SIZE

extern DataLogBuffer dataLogger;

#endif
