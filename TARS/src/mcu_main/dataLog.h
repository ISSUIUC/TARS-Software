#pragma once

#include "common/FifoBuffer.h"
#include "common/MessageQueue.h"
#include "common/packet.h"

#define FIFO_SIZE 200
#define QUEUE_SIZE 5

class DataLogBuffer;
extern DataLogBuffer dataLogger;

class DataLogQueue {
   public:
    friend class DataLogBuffer;
    void attach(DataLogBuffer& buffer);

    sensorDataStruct_t next();

    MessageQueue<LowGData, QUEUE_SIZE> lowGQueue;
    MessageQueue<HighGData, QUEUE_SIZE> highGQueue;
    MessageQueue<GpsData, QUEUE_SIZE> gpsQueue;
    MessageQueue<KalmanData, QUEUE_SIZE> kalmanQueue;
    MessageQueue<rocketStateData<4>, QUEUE_SIZE> rocketStateQueue;
    MessageQueue<BarometerData, QUEUE_SIZE> barometerQueue;
    MessageQueue<GasData, QUEUE_SIZE> gasQueue;
    MessageQueue<MagnetometerData, QUEUE_SIZE> magnetometerQueue;
    MessageQueue<FlapData, QUEUE_SIZE> flapQueue;
    MessageQueue<VoltageData, QUEUE_SIZE> voltageQueue;
    MessageQueue<OrientationData, QUEUE_SIZE> orientationQueue;

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
    FifoBuffer<GasData, FIFO_SIZE> gasFifo;
    FifoBuffer<MagnetometerData, FIFO_SIZE> magnetometerFifo;
    FifoBuffer<FlapData, FIFO_SIZE> flapFifo;
    FifoBuffer<VoltageData, FIFO_SIZE> voltageFifo;
    FifoBuffer<BarometerData, FIFO_SIZE> barometerFifo;
    FifoBuffer<OrientationData, FIFO_SIZE> orientationFifo;

    void pushLowGFifo(LowGData const& lowG_Data);

    void pushHighGFifo(HighGData const& highG_Data);

    void pushGpsFifo(GpsData const& gps_Data);

    void pushKalmanFifo(KalmanData const& state_data);

    void pushRocketStateFifo(rocketStateData<4> const& rocket_data);

    void pushBarometerFifo(BarometerData const& barometer_data);

    void pushGasFifo(GasData const& gas_data);

    void pushMagnetometerFifo(MagnetometerData const& magnetometer_data);

    void pushFlapsFifo(FlapData const& flap_data);

    void pushVoltageFifo(VoltageData const& voltage_data);

    void pushOrientationFifo(OrientationData const& orientation_data);

    sensorDataStruct_t read();
};

#undef FIFO_SIZE
#undef QUEUE_SIZE