#pragma once

//#include <ChRt.h>
//#include <SD.h>

#include "FifoBuffer.h"
#include "MessageQueue.h"
#include "packet.h"

#define FIFO_SIZE 1000
#define QUEUE_SIZE 5

class DataLogBuffer;

extern DataLogBuffer dataLogger;

class DataLogQueue {
public:
    friend class DataLogBuffer;
    void attach(DataLogBuffer& buffer);

    sensorDataStruct_t next();

    MessageQueue<LowGData, QUEUE_SIZE>            lowGQueue;
    MessageQueue<HighGData, QUEUE_SIZE>           highGQueue;
    MessageQueue<GpsData, QUEUE_SIZE>             gpsQueue;
    MessageQueue<KalmanData, QUEUE_SIZE>          kalmanQueue;
    MessageQueue<rocketStateData<4>, QUEUE_SIZE>  rocketStateQueue;
    MessageQueue<BarometerData, QUEUE_SIZE>       barometerQueue;
    MessageQueue<FlapData, QUEUE_SIZE>            flapQueue;
    MessageQueue<VoltageData, QUEUE_SIZE>         voltageQueue;

private:
    DataLogQueue* next_queue = nullptr;
};

/**
 * @brief A class to hold all info for ring buffers and mutexes used for data.
 *
 */
class DataLogBuffer {
    friend class DataLogQueue;

private:
    DataLogQueue* first_queue = nullptr;

public:
    FifoBuffer<LowGData, FIFO_SIZE> lowGFifo;
    FifoBuffer<HighGData, FIFO_SIZE> highGFifo;
    FifoBuffer<GpsData, FIFO_SIZE> gpsFifo;
    FifoBuffer<KalmanData, FIFO_SIZE> kalmanFifo;
    FifoBuffer<rocketStateData<4>, FIFO_SIZE> rocketStateFifo;
    FifoBuffer<FlapData, FIFO_SIZE> flapFifo;
    FifoBuffer<VoltageData, FIFO_SIZE> voltageFifo;
    FifoBuffer<BarometerData, FIFO_SIZE> barometerFifo;

    void pushLowGFifo(LowGData const& lowG_Data);

    void pushHighGFifo(HighGData const& highG_Data);

    void pushGpsFifo(GpsData const& gps_Data);

    void pushKalmanFifo(KalmanData const& state_data);

    void pushRocketStateFifo(rocketStateData<4> const& rocket_data);

    void pushBarometerFifo(BarometerData const& barometer_data);

    void pushFlapsFifo(FlapData const& flap_data);

    void pushVoltageFifo(VoltageData const& voltage_data);

    sensorDataStruct_t read();
};

#undef FIFO_SIZE
#undef QUEUE_SIZE
