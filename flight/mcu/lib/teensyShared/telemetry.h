#include <ChRt.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ServoControl.h>

#include <array>

#include "SD.h"
#include "pins.h"
#include "sensors.h"
#include "FifoBuffer.h"

// Make sure to change these pinout depending on wiring
// Don't forget to change the ini file to build the correct main file

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

#define MAX_CMD_LEN 10

struct TelemetryData2 {
    float gps_lat;
    float gps_long;
    float gps_alt;
    float barometer_pressure;
    float barometer_temp;
    float highG_ax;
    float highG_ay;
    float highG_az;

    int8_t flap_extension;
    int8_t voltage_battery;
    int8_t FSM_State;
    int8_t rssi;
    int response_ID;
    systime_t timestamp;
};

struct TelemetmryPacket {
    TelemetryData2 datapoints[4];
    int datapoint_count;
};

// Data transmitted from rocket to ground station
struct telemetry_data {
    float gps_lat;
    float gps_long;
    float gps_alt;
    float barometer_alt;
    float barometer_temp;
    float barometer_pressure;
    // KX134 (highg) IMU DATA
    float KX_IMU_ax;  // acceleration (in G's)
    float KX_IMU_ay;
    float KX_IMU_az;
    // H3LIS331DL (highg) IMU DATA
    float H3L_IMU_ax;
    float H3L_IMU_ay;
    float H3L_IMU_az;
    // LSM9DS1 (lowg) IMU DATA
    float LSM_IMU_ax;  // acceleration (in G's)
    float LSM_IMU_ay;
    float LSM_IMU_az;
    float LSM_IMU_gx;  // Gyro data (in degrees/sec)
    float LSM_IMU_gy;
    float LSM_IMU_gz;
    float LSM_IMU_mx;
    float LSM_IMU_my;
    float LSM_IMU_mz;

    float flap_extension;
    float voltage_battry;

    float state_x;
    float state_vx;
    float state_ax;
    float state_apo;

    int FSM_state;
    char sign[8] = "HITHERE";
    int rssi;
    float battery_voltage;
    int response_ID;
};

// Commands transmitted from ground station to rocket
enum CommandType { SET_FREQ, SET_CALLSIGN, ABORT, TEST_FLAPS, EMPTY };

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
    Telemetry();
    void transmit();
    void handle_command(const telemetry_command& cmd);
    bool abort = false;
    void buffer_data(const sensorDataStruct_t&);

   private:
    FifoBuffer<TelemetryData2, 4> buffered_data;
    int packetnum;
    telemetry_data d;
    RH_RF95 rf95;

    File read_file;
    File write_file;

    // Initializing command ID
    int last_command_id = -1;

    // Initializing callsign
    char callsign[8] = "NO SIGN";
    command_handler_struct freq_status = {};

    TelemetmryPacket make_packet();
};