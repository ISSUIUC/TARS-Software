/**
 * @file dataLog.cpp
 *
 * Contains the code to handle FIFO buffers for sensors,
 * work with the SD card library. The header file also contains
 * important data structs.
 */

#include "dataLog.h"

DataLogBuffer dataLogger;

/**
 * @brief Reads one sensorDataStruct_t from the contained view and returns it. Also advances the view, naturally.
 *
 */
sensorDataStruct_t DataLogView::read() {
    sensorDataStruct_t data;
    data.has_lowG_data = lowGView.next(data.lowG_data);
    data.has_highG_data = highGView.next(data.highG_data);
    data.has_gps_data = gpsView.next(data.gps_data);
    data.has_kalman_data = kalmanView.next(data.kalman_data);
    data.has_rocketState_data = rocketStateView.next(data.rocketState_data);
    data.has_barometer_data = barometerView.next(data.barometer_data);
    data.has_flap_data = flapView.next(data.flap_data);
    data.has_voltage_data = voltageView.next(data.voltage_data);
    return data;
}

DataLogView::DataLogView(DataLogBuffer& buffer) :
        lowGView(buffer.lowGFifo), highGView(buffer.highGFifo), gpsView(buffer.gpsFifo),
        kalmanView(buffer.kalmanFifo), rocketStateView(buffer.rocketStateFifo), barometerView(buffer.barometerFifo),
        flapView(buffer.flapFifo), voltageView(buffer.voltageFifo)
        { }

void DataLogBuffer::pushLowGFifo(LowGData const& lowG_Data) {
    lowGFifo.push(lowG_Data);
}

void DataLogBuffer::pushHighGFifo(HighGData const& highG_Data) {
    highGFifo.push(highG_Data);
}

void DataLogBuffer::pushGpsFifo(GpsData const& gps_Data) {
    gpsFifo.push(gps_Data);
}

void DataLogBuffer::pushKalmanFifo(KalmanData const& state_data) {
    kalmanFifo.push(state_data);
}

void DataLogBuffer::pushBarometerFifo(BarometerData const& barometer_data) {
    barometerFifo.push(barometer_data);
}

void DataLogBuffer::pushRocketStateFifo(rocketStateData<4> const& rocket_data) {
    rocketStateFifo.push(rocket_data);
}

void DataLogBuffer::pushFlapsFifo(FlapData const& flap_data) {
    flapFifo.push(flap_data);
}

void DataLogBuffer::pushVoltageFifo(VoltageData const& voltage_data) {
    voltageFifo.push(voltage_data);
}