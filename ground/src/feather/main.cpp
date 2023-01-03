/**
 * This file contains the code than runs on our ground station
 * hardware (LoRa Feather module). It includes the receive code,
 * command functionality, and interfaces with the ground station GUI
 * through serial.
 *
 * Spaceshot Telemetry Team 2021-22
 * Nicholas Phillips
 * Gautam Dayal
 * Patrick Marschoun
 * Peter Giannetos
 */

#include <RH_RF95.h>
#include <SPI.h>

#include <array>
#include <limits>
#include <numeric>
#include <queue>

#include "SerialParser.h"

/* Pins for feather*/
// // Ensure to change depending on wiring
#define RFM95_CS 8
#define RFM95_RST 4
// #define RFM95_EN
#define RFM95_INT 3
// #define LED 13 // Blinks on receipt

/* Pins for Teensy 31*/
// Ensure to change depending on wiring
// #define RFM95_CS 10
// #define RFM95_RST 15
// #define RFM95_EN 14
// #define RFM95_INT 16
// #define LED 13 // Blinks on receipt

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

#define DEFAULT_CMD 0
#define MAX_CMD_LEN 10

typedef uint32_t systime_t;
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// For reading from
char incomingCmd[MAX_CMD_LEN];
char curByte;
short charIdx = 0;
short readySend = 0;
int command_ID = 0;
short cmd_number = 0;

template <typename T>
float convert_range(T val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    return val * range / (float)numeric_range;
}

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

struct FullTelemetryData {
    TelemetryDataLite data;
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
    float freq;
    int64_t print_time;
};

struct telemetry_data {
    float gps_lat;
    float gps_long;
    float gps_alt;
    float barometer_alt;
    float barometer_pressure;
    float barometer_temperature;
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
    char sign[8] = "KC1QJA";
    int rssi;
    float battery_voltage;
    int response_ID;
};

enum class CommandType { SET_FREQ, SET_CALLSIGN, ABORT, TEST_FLAP, EMPTY };
// Commands transmitted from ground station to rocket
struct telemetry_command {
    CommandType command;
    int id;
    union {
        char callsign[8];
        float freq;
        bool do_abort;
    };
    std::array<char, 6> verify = {{'A', 'Y', 'B', 'E', 'R', 'K'}};
};

struct TelemetryCommandQueueElement {
    telemetry_command command;
    int retry_count;
};

std::queue<TelemetryCommandQueueElement> cmd_queue;
std::queue<FullTelemetryData> print_queue;

constexpr const char* json_command_success = R"({"type": "command_success"})";
constexpr const char* json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
constexpr const char* json_buffer_full_error = R"({"type": "command_error", "error": "command buffer not empty"})";

constexpr const char* json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
constexpr const char* json_init_success = R"({"type": "init_success"})";
constexpr const char* json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
constexpr const char* json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
constexpr const char* json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";
constexpr int max_command_retries = 5;

float current_freq = RF95_FREQ;

void printFloat(float f, int precision = 5) {
    if (isinf(f) || isnan(f)) {
        Serial.print(-1);
    } else {
        Serial.print(f, precision);
    }
}

void EnqueuePacket(const TelemetryPacket& packet, float frequency) {
    if (packet.datapoint_count == 0) return;

    int64_t start_timestamp = packet.datapoints[0].timestamp;
    int64_t start_printing = millis();

    for (int i = 0; i < packet.datapoint_count && i < 4; i++) {
        FullTelemetryData item;
        item.gps_alt = packet.gps_alt;
        item.gps_lat = packet.gps_lat;
        item.gps_long = packet.gps_long;
        item.freq = frequency;
        item.FSM_State = packet.FSM_State;
        item.gnc_state_ax = packet.gnc_state_ax;
        item.gnc_state_vx = packet.gnc_state_vx;
        item.gnc_state_x = packet.gnc_state_x;
        item.gns_state_apo = packet.gns_state_apo;
        item.response_ID = packet.response_ID;
        item.rssi = packet.rssi;
        item.voltage_battery = packet.voltage_battery;
        item.data = packet.datapoints[i];
        item.print_time = start_printing - start_timestamp + item.data.timestamp;
        print_queue.emplace(item);
        // // space packets out
        // int64_t time_diff = packet.datapoints[i].timestamp - start_timestamp;
        // if (time_diff > 0) {
        //     int64_t elapsed = millis() - start_printing;

        //     if (elapsed < time_diff) {
        //         // failsafe if sleep time is too long
        //         delay(std::min(time_diff - elapsed, 200ll));
        //     }
        // }

        // TelemetryDataLite data = packet.datapoints[i];
    }
}

void PrintDequeue() {
    if (print_queue.empty()) return;

    auto packet = print_queue.front();
    if (packet.print_time > millis()) return;
    print_queue.pop();

    TelemetryDataLite data = packet.data;

    float baro_altitude = -log(convert_range(data.barometer_pressure, 4096) * 0.000987) *
                          (convert_range(data.barometer_temp, 128) + 273.15) * 29.254;
    Serial.print(R"({"type": "data", "value": {)");
    Serial.print(R"("response_ID":)");
    Serial.print(packet.response_ID);
    Serial.print(',');
    Serial.print(R"("gps_lat":)");
    printFloat(packet.gps_lat);
    Serial.print(',');
    Serial.print(R"("gps_long":)");
    printFloat(packet.gps_long);
    Serial.print(',');
    Serial.print(R"("gps_alt":)");
    printFloat(packet.gps_alt);
    Serial.print(',');
    Serial.print(R"("barometer_alt":)");
    printFloat(baro_altitude);
    Serial.print(',');
    Serial.print(R"("KX_IMU_ax":)");
    printFloat(convert_range(data.highG_ax, 256));
    Serial.print(',');
    Serial.print(R"("KX_IMU_ay":)");
    printFloat(convert_range(data.highG_ay, 256));
    Serial.print(',');
    Serial.print(R"("KX_IMU_az":)");
    printFloat(convert_range(data.highG_az, 256));
    Serial.print(',');
    // Serial.print(R"("H3L_IMU_ax":)"); Serial.print(data.H3L_IMU_ax);
    // Serial.print(','); Serial.print(R"("H3L_IMU_ay":)");
    // Serial.print(data.H3L_IMU_ay); Serial.print(',');
    // Serial.print(R"("H3L_IMU_az":)"); Serial.print(data.H3L_IMU_az);
    // Serial.print(',');
    Serial.print(R"("LSM_IMU_ax":)");
    printFloat(0);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_ay":)");
    printFloat(0);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_az":)");
    printFloat(0);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_gx":)");
    printFloat(convert_range(data.gyro_x, 8192));
    Serial.print(',');
    Serial.print(R"("LSM_IMU_gy":)");
    printFloat(convert_range(data.gyro_y, 8192));
    Serial.print(',');
    Serial.print(R"("LSM_IMU_gz":)");
    printFloat(convert_range(data.gyro_z, 8192));
    Serial.print(',');
    Serial.print(R"("LSM_IMU_mx":)");
    printFloat(0);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_my":)");
    printFloat(0);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_mz":)");
    printFloat(0);
    Serial.print(',');
    Serial.print(R"("FSM_state":)");
    Serial.print(packet.FSM_State);
    Serial.print(',');
    Serial.print(R"("sign":")");
    Serial.print("NOSIGN");
    Serial.print("\",");
    Serial.print(R"("RSSI":)");
    Serial.print(rf95.lastRssi());
    Serial.print(',');
    Serial.print(R"("Voltage":)");
    printFloat(convert_range(packet.voltage_battery, 16));
    Serial.print(',');
    Serial.print(R"("frequency":)");
    Serial.print(packet.freq);
    Serial.print(',');
    Serial.print(R"("flap_extension":)");
    printFloat(data.flap_extension);
    Serial.print(",");
    Serial.print(R"("STE_ALT":)");
    printFloat(packet.gnc_state_x);
    Serial.print(",");
    Serial.print(R"("STE_VEL":)");
    printFloat(packet.gnc_state_vx);
    Serial.print(",");
    Serial.print(R"("STE_ACC":)");
    printFloat(packet.gnc_state_ax);
    Serial.print(",");
    Serial.print(R"("TEMP":)");
    printFloat(convert_range(data.barometer_temp, 128));
    Serial.print(",");
    Serial.print(R"("pressure":)");
    printFloat(convert_range(data.barometer_pressure, 4096));
    Serial.print(",");
    Serial.print(R"("STE_APO":)");
    printFloat(packet.gns_state_apo);
    Serial.print("");

    Serial.println("}}");
}

void SerialPrintTelemetryData(const TelemetryPacket& packet, float frequency) {
    if (packet.datapoint_count == 0) return;

    int64_t start_timestamp = packet.datapoints[0].timestamp;
    int64_t start_printing = millis();

    for (int i = 0; i < packet.datapoint_count && i < 4; i++) {
        // space packets out
        int64_t time_diff = packet.datapoints[i].timestamp - start_timestamp;
        if (time_diff > 0) {
            int64_t elapsed = millis() - start_printing;

            if (elapsed < time_diff) {
                // failsafe if sleep time is too long
                delay(std::min(time_diff - elapsed, 200ll));
            }
        }

        TelemetryDataLite data = packet.datapoints[i];

        float baro_altitude = -log(convert_range(data.barometer_pressure, 4096) * 0.000987) *
                              (convert_range(data.barometer_temp, 128) + 273.15) * 29.254;
        Serial.print(R"({"type": "data", "value": {)");
        Serial.print(R"("response_ID":)");
        Serial.print(packet.response_ID);
        Serial.print(',');
        Serial.print(R"("gps_lat":)");
        printFloat(packet.gps_lat);
        Serial.print(',');
        Serial.print(R"("gps_long":)");
        printFloat(packet.gps_long);
        Serial.print(',');
        Serial.print(R"("gps_alt":)");
        printFloat(packet.gps_alt);
        Serial.print(',');
        Serial.print(R"("barometer_alt":)");
        printFloat(baro_altitude);
        Serial.print(',');
        Serial.print(R"("KX_IMU_ax":)");
        printFloat(convert_range(data.highG_ax, 256));
        Serial.print(',');
        Serial.print(R"("KX_IMU_ay":)");
        printFloat(convert_range(data.highG_ay, 256));
        Serial.print(',');
        Serial.print(R"("KX_IMU_az":)");
        printFloat(convert_range(data.highG_az, 256));
        Serial.print(',');
        // Serial.print(R"("H3L_IMU_ax":)"); Serial.print(data.H3L_IMU_ax);
        // Serial.print(','); Serial.print(R"("H3L_IMU_ay":)");
        // Serial.print(data.H3L_IMU_ay); Serial.print(',');
        // Serial.print(R"("H3L_IMU_az":)"); Serial.print(data.H3L_IMU_az);
        // Serial.print(',');
        Serial.print(R"("LSM_IMU_ax":)");
        printFloat(0);
        Serial.print(',');
        Serial.print(R"("LSM_IMU_ay":)");
        printFloat(0);
        Serial.print(',');
        Serial.print(R"("LSM_IMU_az":)");
        printFloat(0);
        Serial.print(',');
        Serial.print(R"("LSM_IMU_gx":)");
        printFloat(convert_range(data.gyro_x, 8192));
        Serial.print(',');
        Serial.print(R"("LSM_IMU_gy":)");
        printFloat(convert_range(data.gyro_y, 8192));
        Serial.print(',');
        Serial.print(R"("LSM_IMU_gz":)");
        printFloat(convert_range(data.gyro_z, 8192));
        Serial.print(',');
        Serial.print(R"("LSM_IMU_mx":)");
        printFloat(0);
        Serial.print(',');
        Serial.print(R"("LSM_IMU_my":)");
        printFloat(0);
        Serial.print(',');
        Serial.print(R"("LSM_IMU_mz":)");
        printFloat(0);
        Serial.print(',');
        Serial.print(R"("FSM_state":)");
        Serial.print(packet.FSM_State);
        Serial.print(',');
        Serial.print(R"("sign":")");
        Serial.print("NOSIGN");
        Serial.print("\",");
        Serial.print(R"("RSSI":)");
        Serial.print(rf95.lastRssi());
        Serial.print(',');
        Serial.print(R"("Voltage":)");
        printFloat(convert_range(packet.voltage_battery, 16));
        Serial.print(',');
        Serial.print(R"("frequency":)");
        Serial.print(frequency);
        Serial.print(',');
        Serial.print(R"("flap_extension":)");
        printFloat(data.flap_extension);
        Serial.print(",");
        Serial.print(R"("STE_ALT":)");
        printFloat(packet.gnc_state_x);
        Serial.print(",");
        Serial.print(R"("STE_VEL":)");
        printFloat(packet.gnc_state_vx);
        Serial.print(",");
        Serial.print(R"("STE_ACC":)");
        printFloat(packet.gnc_state_ax);
        Serial.print(",");
        Serial.print(R"("TEMP":)");
        printFloat(convert_range(data.barometer_temp, 128));
        Serial.print(",");
        Serial.print(R"("pressure":)");
        printFloat(convert_range(data.barometer_pressure, 4096));
        Serial.print(",");
        Serial.print(R"("STE_APO":)");
        printFloat(packet.gns_state_apo);
        Serial.print("");

        Serial.println("}}");
    }
    // add null ternimator to sign
    // char sign[9]{};
    // memcpy(sign, data.sign, 8);
}

void SerialError() { Serial.println(json_command_parse_error); }

void set_freq_local_bug_fix(float freq) {
    telemetry_command t;
    t.command = CommandType::EMPTY;
    rf95.send((uint8_t*)&t, sizeof(t));
    rf95.waitPacketSent();
    rf95.setFrequency(freq);
    current_freq = freq;
}

void SerialInput(const char* key, const char* value) {
    /* If queue is not empty, do not accept new command*/
    if (!cmd_queue.empty()) {
        Serial.println(json_buffer_full_error);
        return;
    }

    telemetry_command command{};
    if (strcmp(key, "ABORT") == 0) {
        command.command = CommandType::ABORT;
        command.do_abort = true;
    } else if (strcmp(key, "FREQ") == 0) {
        command.command = CommandType::SET_FREQ;
        float v = atof(value);
        command.freq = min(max(v, 390), 445);
    } else if (strcmp(key, "CALLSIGN") == 0) {
        command.command = CommandType::SET_CALLSIGN;
        memset(command.callsign, ' ', sizeof(command.callsign));
        memcpy(command.callsign, value, min(strlen(value), sizeof(command.callsign)));
    } else if (strcmp(key, "FLOC") == 0) {
        float v = atof(value);
        v = min(max(v, 390), 445);
        set_freq_local_bug_fix(v);
        Serial.println(json_command_success);
        Serial.print(R"({"type": "freq_success", "frequency":)");
        Serial.print(v);
        Serial.println("}");
        return;
    } else if (strcmp(key, "FLAP") == 0) {
        command.command = CommandType::TEST_FLAP;
    } else {
        SerialError();
        return;
    }
    Serial.println(json_command_success);
    command_ID++;
    command.id = command_ID;
    cmd_queue.push({command, 0});
}

void process_command_queue() {
    if (cmd_queue.empty()) return;

    TelemetryCommandQueueElement cmd = cmd_queue.front();
    rf95.send((uint8_t*)&cmd.command, sizeof(cmd.command));
    rf95.waitPacketSent();
}

SerialParser serial_parser(SerialInput, SerialError);

void setup() {
    while (!Serial)
        ;
    Serial.begin(9600);

    if (!rf95.init()) {
        Serial.println(json_init_failure);
        while (1)
            ;
    }

    Serial.println(json_init_success);

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println(json_set_frequency_failure);
        while (1)
            ;
    }

    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(RF95_FREQ);
    Serial.println("}");

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
    // transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
}

void loop() {
    PrintDequeue();
    if (rf95.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        // telemetry_data data{};
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            memcpy(&packet, buf, sizeof(packet));
            EnqueuePacket(packet, current_freq);
            // SerialPrintTelemetryData(packet, current_freq);

            if (!cmd_queue.empty()) {
                auto& cmd = cmd_queue.front();
                if (cmd.command.id == packet.response_ID) {
                    if (cmd.command.command == CommandType::SET_FREQ) {
                        set_freq_local_bug_fix(cmd.command.freq);
                        Serial.print(R"({"type": "freq_success", "frequency":)");
                        Serial.print(cmd.command.freq);
                        Serial.println("}");
                    }
                    cmd_queue.pop();
                } else {
                    cmd.retry_count++;
                    if (cmd.retry_count >= max_command_retries) {
                        cmd_queue.pop();
                        Serial.println(json_send_failure);
                    }
                }
            }

            process_command_queue();

        } else {
            Serial.println(json_receive_failure);
        }
    }
    serial_parser.read();
}