/**
 * @file dataLog.cpp
 *
 * Contains the code to handle FIFO buffers for sensors,
 * work with the SD card library. The header file also contains
 * important data structs.
 */

#include "mcu_main/dataLog.h"

DataLogBuffer dataLogger;

// sensorDataStruct_t DataLogView::read() {
//     sensorDataStruct_t data;
//     data.has_lowG_data = lowGView.next(data.lowG_data);
//     data.has_highG_data = highGView.next(data.highG_data);
//     data.has_gps_data = gpsView.next(data.gps_data);
//     data.has_kalman_data = kalmanView.next(data.kalman_data);
//     data.has_rocketState_data = rocketStateView.next(data.rocketState_data);
//     data.has_barometer_data = barometerView.next(data.barometer_data);
//     data.has_flap_data = flapView.next(data.flap_data);
//     data.has_voltage_data = voltageView.next(data.voltage_data);
//     return data;
// }

// DataLogView::DataLogView(DataLogBuffer& buffer) :
//         lowGView(buffer.lowGFifo), highGView(buffer.highGFifo), gpsView(buffer.gpsFifo),
//         kalmanView(buffer.kalmanFifo), rocketStateView(buffer.rocketStateFifo), barometerView(buffer.barometerFifo),
//         flapView(buffer.flapFifo), voltageView(buffer.voltageFifo)
//         { }

sensorDataStruct_t DataLogBuffer::read() {
    sensorDataStruct_t data;
    lowGFifo.read(data.lowG_data);
    highGFifo.read(data.highG_data);
    gpsFifo.read(data.gps_data);
    kalmanFifo.read(data.kalman_data);
    rocketStateFifo.read(data.rocketState_data);
    barometerFifo.read(data.barometer_data);
    flapFifo.read(data.flap_data);
    voltageFifo.read(data.voltage_data);
    orientationFifo.read(data.orientation_data);
    magnetometerFifo.read(data.magnetometer_data);
    gasFifo.read(data.gas_data);
    return data;
}

#define UPDATE_QUEUE(queue, data)          \
    do {                                   \
        DataLogQueue* curr_ = first_queue; \
        while (curr_) {                    \
            curr_->queue.push((data));     \
            curr_ = curr_->next_queue;     \
        }                                  \
    } while (false)

void DataLogBuffer::pushLowGFifo(LowGData const& lowG_Data) {
    lowGFifo.push(lowG_Data);
    UPDATE_QUEUE(lowGQueue, lowG_Data);
}

void DataLogBuffer::pushHighGFifo(HighGData const& highG_Data) {
    highGFifo.push(highG_Data);
    UPDATE_QUEUE(highGQueue, highG_Data);
}

void DataLogBuffer::pushGpsFifo(GpsData const& gps_Data) {
    gpsFifo.push(gps_Data);
    UPDATE_QUEUE(gpsQueue, gps_Data);
}

void DataLogBuffer::pushKalmanFifo(KalmanData const& state_data) {
    kalmanFifo.push(state_data);
    UPDATE_QUEUE(kalmanQueue, state_data);
}

void DataLogBuffer::pushBarometerFifo(BarometerData const& barometer_data) {
    barometerFifo.push(barometer_data);
    UPDATE_QUEUE(barometerQueue, barometer_data);
}

void DataLogBuffer::pushRocketStateFifo(rocketStateData<4> const& rocket_data) {
    rocketStateFifo.push(rocket_data);
    UPDATE_QUEUE(rocketStateQueue, rocket_data);
}

void DataLogBuffer::pushFlapsFifo(FlapData const& flap_data) {
    flapFifo.push(flap_data);
    UPDATE_QUEUE(flapQueue, flap_data);
}

void DataLogBuffer::pushVoltageFifo(VoltageData const& voltage_data) {
    voltageFifo.push(voltage_data);
    UPDATE_QUEUE(voltageQueue, voltage_data);
}

void DataLogBuffer::pushOrientationFifo(OrientationData const& orientation_data) {
    orientationFifo.push(orientation_data);
    UPDATE_QUEUE(orientationQueue, orientation_data);
}

void DataLogBuffer::pushGasFifo(const GasData& gas_data) {
    gasFifo.push(gas_data);
    UPDATE_QUEUE(gasQueue, gas_data);
}

void DataLogBuffer::pushMagnetometerFifo(const MagnetometerData& magnetometer_data) {
    magnetometerFifo.push(magnetometer_data);
    UPDATE_QUEUE(magnetometerQueue, magnetometer_data);
}

#undef UPDATE_QUEUE

void DataLogQueue::attach(DataLogBuffer& buffer) {
    next_queue = buffer.first_queue;
    buffer.first_queue = this;
}

sensorDataStruct_t DataLogQueue::next() {
    sensorDataStruct_t data;
    data.has_lowG_data = lowGQueue.pop(data.lowG_data);
    data.has_highG_data = highGQueue.pop(data.highG_data);
    data.has_gps_data = gpsQueue.pop(data.gps_data);
    data.has_kalman_data = kalmanQueue.pop(data.kalman_data);
    data.has_rocketState_data = rocketStateQueue.pop(data.rocketState_data);
    data.has_barometer_data = barometerQueue.pop(data.barometer_data);
    data.has_flap_data = flapQueue.pop(data.flap_data);
    data.has_voltage_data = voltageQueue.pop(data.voltage_data);
    data.has_orientation_data = orientationQueue.pop(data.orientation_data);
    data.has_gas_data = gasQueue.pop(data.gas_data);
    data.has_magnetometer_data = magnetometerQueue.pop(data.magnetometer_data);
    return data;
}