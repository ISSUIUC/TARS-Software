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
    int16_t bno_roll;             //[-4,4]
    int16_t bno_pitch;            //[-4,4]
    int16_t bno_yaw;              //[-4,4]

    float flap_extension;
};

struct TelemetryPacket {
    TelemetryDataLite datapoints[4];
    float gps_lat;
    float gps_long;
    float gps_alt;
    float yaw;
    float pitch;
    float roll;
    float gnc_state_x;
    float gnc_state_vx;
    float gnc_state_ax;
    float gnc_state_y;
    float gnc_state_vy;
    float gnc_state_ay;
    float gnc_state_z;
    float gnc_state_vz;
    float gnc_state_az;
    float gnc_state_apo;
    int16_t mag_x;            //[-4, 4]
    int16_t mag_y;            //[-4, 4]
    int16_t mag_z;            //[-4, 4]
    int16_t gyro_x;           //[-4096, 4096]
    int16_t gyro_y;           //[-4096, 4096]
    int16_t gyro_z;           //[-4096, 4096]
    int16_t response_ID;      //[0, 2^16]
    int8_t rssi;              //[-128, 128]
    int8_t datapoint_count;   //[0,4]
    uint8_t voltage_battery;  //[0, 16]
    uint8_t FSM_State;        //[0,256]
    int16_t barometer_temp;   //[-128, 128]
};

struct FullTelemetryData {
    systime_t timestamp;  //[0, 2^32]

    float barometer_pressure;  //[0, 4096]
    float highG_ax;            //[128, -128]
    float highG_ay;            //[128, -128]
    float highG_az;            //[128, -128]
    float bno_yaw;             //[-4,4]
    float bno_pitch;           //[-4,4]
    float bno_roll;            //[-4,4]

    float flap_extension;  //[0, 256]
    float gps_lat;
    float gps_long;
    float gps_alt;
    float gnc_state_x;
    float gnc_state_vx;
    float gnc_state_ax;
    float gnc_state_apo;
    float mag_x;            //[-4, 4]
    float mag_y;            //[-4, 4]
    float mag_z;            //[-4, 4]
    float gyro_x;           //[-4096, 4096]
    float gyro_y;           //[-4096, 4096]
    float gyro_z;           //[-4096, 4096]
    int16_t response_ID;    //[0, 2^16]
    int8_t rssi;            //[-128, 128]
    float voltage_battery;  //[0, 16]
    uint8_t FSM_State;      //[0,256]
    float barometer_temp;   //[-128, 128]
    float freq;
    int64_t print_time;
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
        TelemetryDataLite data = packet.datapoints[i];
        item.barometer_pressure = convert_range(data.barometer_pressure, 4096);
        item.barometer_temp = convert_range(packet.barometer_temp, 256);
        item.highG_ax = convert_range(data.highG_ax, 256);
        item.highG_ay = convert_range(data.highG_ay, 256);
        item.highG_az = convert_range(data.highG_az, 256);
        item.gyro_x = convert_range(packet.gyro_x, 8192);
        item.gyro_y = convert_range(packet.gyro_y, 8192);
        item.gyro_z = convert_range(packet.gyro_z, 8192);
        item.bno_roll = convert_range(data.bno_roll, 8);
        item.bno_pitch = convert_range(data.bno_pitch, 8);
        item.bno_yaw = convert_range(data.bno_yaw, 8);
        item.mag_x = convert_range(packet.mag_x, 8);
        item.mag_y = convert_range(packet.mag_y, 8);
        item.mag_z = convert_range(packet.mag_z, 8);
        item.flap_extension = data.flap_extension;
        item.gps_alt = packet.gps_alt;
        item.gps_lat = packet.gps_lat;
        item.gps_long = packet.gps_long;
        item.freq = frequency;
        item.FSM_State = packet.FSM_State;
        item.gnc_state_ax = packet.gnc_state_ax;
        item.gnc_state_vx = packet.gnc_state_vx;
        item.gnc_state_x = packet.gnc_state_x;
        item.gnc_state_apo = packet.gnc_state_apo;
        item.response_ID = packet.response_ID;
        item.rssi = packet.rssi;
        item.voltage_battery = convert_range(packet.voltage_battery, 16);
        item.print_time = start_printing - start_timestamp + data.timestamp;
        print_queue.emplace(item);
    }
}

void printJSONField(const char* name, float val, bool comma = true) {
    Serial.print('\"');
    Serial.print(name);
    Serial.print("\":");
    printFloat(val);
    if (comma) Serial.print(',');
}

void printJSONField(const char* name, int val, bool comma = true) {
    Serial.print('\"');
    Serial.print(name);
    Serial.print("\":");
    Serial.print(val);
    if (comma) Serial.print(',');
}

void printJSONField(const char* name, const char* val, bool comma = true) {
    Serial.print('\"');
    Serial.print(name);
    Serial.print("\":\"");
    Serial.print(val);
    Serial.print('"');
    if (comma) Serial.print(',');
}

void printPacketJson(FullTelemetryData const& packet) {
    Serial.print(R"({"type": "data", "value": {)");
    printJSONField("response_ID", packet.response_ID);
    printJSONField("gps_lat", packet.gps_lat);
    printJSONField("gps_long", packet.gps_long);
    printJSONField("gps_alt", packet.gps_alt);
    printJSONField("KX_IMU_ax", packet.highG_ax);
    printJSONField("KX_IMU_ay", packet.highG_ay);
    printJSONField("KX_IMU_az", packet.highG_az);
    printJSONField("IMU_gx", packet.gyro_x);
    printJSONField("IMU_gy", packet.gyro_y);
    printJSONField("IMU_gz", packet.gyro_z);
    printJSONField("IMU_mx", packet.mag_x);
    printJSONField("IMU_my", packet.mag_y);
    printJSONField("IMU_mz", packet.mag_z);
    printJSONField("FSM_state", packet.FSM_State);
    printJSONField("sign", "NOSIGN");
    printJSONField("RSSI", rf95.lastRssi());
    printJSONField("Voltage", packet.voltage_battery);
    printJSONField("frequency", packet.freq);
    printJSONField("flap_extension", packet.flap_extension);
    printJSONField("STE_ALT", packet.gnc_state_x);
    printJSONField("STE_VEL", packet.gnc_state_vx);
    printJSONField("STE_ACC", packet.gnc_state_ax);
    printJSONField("STE_APO", packet.gnc_state_apo);
    printJSONField("BNO_YAW", packet.bno_yaw);
    printJSONField("BNO_PITCH", packet.bno_pitch);
    printJSONField("BNO_ROLL", packet.bno_roll);
    printJSONField("TEMP", packet.barometer_temp);
    printJSONField("pressure", packet.barometer_pressure, false);
    Serial.println("}}");
}

void PrintDequeue() {
    if (print_queue.empty()) return;

    auto packet = print_queue.front();
    if (packet.print_time > millis()) return;
    print_queue.pop();
    printPacketJson(packet);
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

    pinMode(LED_BUILTIN, OUTPUT);
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
    // static float f = 0;
    // static float f2 = 0;
    // f+=0.1;
    // f2 += 0.01;
    // if(f > 3.14) f -= 6.28;
    // delay(30);
    // FullTelemetryData d{};
    // d.barometer_pressure = 1000;
    // d.barometer_temp = 20;
    // d.bno_pitch = cos(f2);
    // d.bno_roll = sin(f2);
    // d.bno_yaw = 0;
    // d.flap_extension = f / 10;
    // d.freq = 434;
    // d.FSM_State = 3;
    // d.gnc_state_ax = f * 100;
    // d.gnc_state_vx = f * 10;
    // d.gnc_state_x = f * 1000;
    // d.gnc_state_apo = 100;
    // d.gps_alt = 1000+100*f;
    // d.gps_lat = 40;
    // d.gps_long = 80 + f;
    // d.gyro_x = sin(f2);
    // d.gyro_y = f+30;
    // d.gyro_z = f+40;
    // d.highG_ax = 10+f;
    // d.highG_ay = f/10;
    // d.highG_az = f/10;
    // d.mag_x = f;
    // d.mag_y = f+1;
    // d.mag_z = f+2;
    // d.voltage_battery = f + 4;
    // printPacketJson(d);

    if (rf95.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        // telemetry_data data{};
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len) && len == sizeof(packet)) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            memcpy(&packet, buf, sizeof(packet));
            EnqueuePacket(packet, current_freq);

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