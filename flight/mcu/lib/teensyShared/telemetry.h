#pragma once

#include <ChRt.h>
#include <RH_RF95.h>
#include <array>

#include "pins.h"
#include "packet.h"
#include "FifoBuffer.h"

// Make sure to change these pinout depending on wiring
// Don't forget to change the ini file to build the correct main file

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

class Telemetry;
extern Telemetry tlm;


struct TelemetryDataLite {
    systime_t timestamp;  //[0, 2^32]

    uint16_t barometer_pressure;  //[0, 4096]
    int16_t highG_ax;             //[128, -128]
    int16_t highG_ay;             //[128, -128]
    int16_t highG_az;             //[128, -128]
    int16_t gyro_x;               //[-4096, 4096]
    int16_t gyro_y;               //[-4096, 4096]
    int16_t gyro_z;               //[-4096, 4096]

    uint8_t flap_extension;  //[0, 256]
    uint8_t barometer_temp;  //[0, 128]
};

struct TelemetryPacket {
    TelemetryDataLite datapoints[4];
    float gps_lat;
    float gps_long;
    float gps_alt;
    float gnc_state_x;
    float gnc_state_vx;
    float gnc_state_ax;
    float gns_state_apo;
    int16_t response_ID;      //[0, 2^16]
    int8_t rssi;              //[-128, 128]
    int8_t datapoint_count;   //[0,4]
    uint8_t voltage_battery;  //[0, 16]
    uint8_t FSM_State;        //[0,256]
};


// Commands transmitted from ground station to rocket
enum CommandType {
    SET_FREQ, SET_CALLSIGN, ABORT, TEST_FLAPS, EMPTY
};

struct telemetry_command {
    CommandType command;
    int cmd_id;
    union {
        char callsign[8];
        float freq;
        bool do_abort;
    };
    std::array<char, 6> verify;
};

struct command_handler_struct {
    bool should_change{};
    float new_freq{};
};

class Telemetry {
public:
    bool abort = false;

    Telemetry();

    void init();

    void transmit();

    void handle_command(const telemetry_command& cmd);

    void bufferData();

    void serial_print(const sensorDataStruct_t&);

private:
    RH_RF95 rf95;
    FifoBuffer<TelemetryDataLite, 4> buffered_data;
    FifoView<TelemetryDataLite, 4> data_view;

    // Initializing command ID
    int16_t last_command_id = -1;

    // Initializing callsign
    char callsign[8] = "NO SIGN";
    command_handler_struct freq_status = {};

    TelemetryPacket make_packet(const sensorDataStruct_t&);
};
