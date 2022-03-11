// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX



/*
This code was used to test the RFM LoRa modules on a breadboard:
    - Frequency: 434Hz
    - Make sure to change Teensy version in platformio
    - Make sure to exclude/include the appropriate files to build in platformio.ini
    - Current problem: Only one-way communication
*/

#include <SPI.h>
#include <RH_RF95.h>
#include "SerialParser.h"
#include <queue>

/* Pins for feather*/
// // Ensure to change depending on wiring
#define RFM95_CS 8
#define RFM95_RST 4
// #define RFM95_EN 
#define RFM95_INT 3
// // #define LED 13 // Blinks on receipt

/* Pins for Teensy 31*/
// Ensure to change depending on wiring
// #define RFM95_CS 10
// #define RFM95_RST 15
// #define RFM95_EN 14
// #define RFM95_INT 16
#define LED 13 // Blinks on receipt

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 440.0

#define DEFAULT_CMD 0
#define MAX_CMD_LEN 10

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//For reading from 
char incomingCmd[MAX_CMD_LEN];
char curByte;
short charIdx = 0;
short readySend = 0;
int command_ID = 0;
short cmd_number = 0;

struct telemetry_data {
  double gps_lat{};
  double gps_long{};
  double gps_alt{};
  double barometer_alt{};
  // KX134 (highg) IMU DATA
  double KX_IMU_ax{};   // acceleration (in G's)
  double KX_IMU_ay{};
  double KX_IMU_az{};
  // H3LIS331DL (highg) IMU DATA
  double H3L_IMU_ax{};
  double H3L_IMU_ay{};
  double H3L_IMU_az{};
  // LSM9DS1 (lowg) IMU DATA
  double LSM_IMU_ax{};    // acceleration (in G's)
  double LSM_IMU_ay{};
  double LSM_IMU_az{};
  double LSM_IMU_gx{};    // Gyro data (in degrees/sec)
  double LSM_IMU_gy{};
  double LSM_IMU_gz{};
  double LSM_IMU_mx{};
  double LSM_IMU_my{};
  double LSM_IMU_mz{};
  
  int FSM_state{};
  char sign[8] = "KC1QJA";
  int rssi;
  double battery_voltage;
  int response_ID;

};


enum class CommandType {
  SET_FREQ,
  SET_CALLSIGN,
  ABORT,
  EMPTY
};
// Commands transmitted from ground station to rocket
struct telemetry_command {
  CommandType command;
  int id;
  union {
    char callsign[8];
    int freq;
    bool do_abort;
  };
};

struct TelemetryCommandQueueElement {
  telemetry_command command;
  int retry_count;
};

std::queue<TelemetryCommandQueueElement> cmd_queue;

constexpr const char * json_command_success = R"({"type": "command_success"})";
constexpr const char * json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
constexpr const char * json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
constexpr const char * json_init_success = R"({"type": "init_success"})";
constexpr const char * json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
constexpr const char * json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
constexpr const char * json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";
constexpr int max_command_retries = 5;

void SerialPrintTelemetryData(const telemetry_data & data){
  //add null ternimator to sign
  char sign[9]{};
  memcpy(sign, data.sign, 8);

  Serial.print(R"({"type": "data", "value": {)");
  Serial.print(R"("response_ID":)"); Serial.print(data.response_ID); Serial.print(',');
  Serial.print(R"("gps_lat":)"); Serial.print(data.gps_lat); Serial.print(',');
  Serial.print(R"("gps_long":)"); Serial.print(data.gps_long); Serial.print(',');
  Serial.print(R"("gps_alt":)"); Serial.print(data.gps_alt); Serial.print(',');
  Serial.print(R"("barometer_alt":)"); Serial.print(data.barometer_alt); Serial.print(',');
  Serial.print(R"("KX_IMU_ax":)"); Serial.print(data.KX_IMU_ax); Serial.print(',');
  Serial.print(R"("KX_IMU_ay":)"); Serial.print(data.KX_IMU_ay); Serial.print(',');
  Serial.print(R"("KX_IMU_az":)"); Serial.print(data.KX_IMU_az); Serial.print(',');
  Serial.print(R"("H3L_IMU_ax":)"); Serial.print(data.H3L_IMU_ax); Serial.print(',');
  Serial.print(R"("H3L_IMU_ay":)"); Serial.print(data.H3L_IMU_ay); Serial.print(',');
  Serial.print(R"("H3L_IMU_az":)"); Serial.print(data.H3L_IMU_az); Serial.print(',');
  Serial.print(R"("LSM_IMU_ax":)"); Serial.print(data.LSM_IMU_ax); Serial.print(',');
  Serial.print(R"("LSM_IMU_ay":)"); Serial.print(data.LSM_IMU_ay); Serial.print(',');
  Serial.print(R"("LSM_IMU_az":)"); Serial.print(data.LSM_IMU_az); Serial.print(',');
  Serial.print(R"("LSM_IMU_gx":)"); Serial.print(data.LSM_IMU_gx); Serial.print(',');
  Serial.print(R"("LSM_IMU_gy":)"); Serial.print(data.LSM_IMU_gy); Serial.print(',');
  Serial.print(R"("LSM_IMU_gz":)"); Serial.print(data.LSM_IMU_gz); Serial.print(',');
  Serial.print(R"("LSM_IMU_mx":)"); Serial.print(data.LSM_IMU_mx); Serial.print(',');
  Serial.print(R"("LSM_IMU_my":)"); Serial.print(data.LSM_IMU_my); Serial.print(',');
  Serial.print(R"("LSM_IMU_mz":)"); Serial.print(data.LSM_IMU_mz); Serial.print(',');
  Serial.print(R"("FSM_state":)"); Serial.print(data.FSM_state); Serial.print(',');
  Serial.print(R"("sign":")"); Serial.print(sign); Serial.print("\",");
  Serial.print(R"("RSSI":)"); Serial.print(rf95.lastRssi()); Serial.print("");

  Serial.println("}}");
}


void SerialError(){
  Serial.println(json_command_parse_error);
}

void set_freq_local_bug_fix(float freq){
  telemetry_command t;
  t.command = CommandType::EMPTY;
  rf95.send((uint8_t*)&t, sizeof(t));
  rf95.waitPacketSent();
  rf95.setFrequency(freq);
}

void SerialInput(const char * key, const char * value){

  /* If queue is not empty, do not accept new command*/
  if(!cmd_queue.empty()){
    SerialError();
    return;
  }

  telemetry_command command{};
  if(strcmp(key, "ABORT") == 0){
    command.command = CommandType::ABORT;
    command.do_abort = true;
  } else if(strcmp(key, "FREQ") == 0) {
    command.command = CommandType::SET_FREQ;
    int v = atoi(value);
    command.freq = min(max(v, 390), 445);
  } else if(strcmp(key, "CALLSIGN") == 0) {
    command.command = CommandType::SET_CALLSIGN;
    memcpy(command.callsign, value, 8);
  } else if(strcmp(key, "FLOC") == 0){
    int v = atoi(value);
    v = min(max(v, 390), 445);
    set_freq_local_bug_fix(v);;
    Serial.println(json_command_success);
    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(v);
    Serial.println("}");
    return;
  } else {
    SerialError();
    return;
  }
  Serial.println(json_command_success);
  command_ID++;
  command.id = command_ID;
  cmd_queue.push({command, 0});
}

void process_command_queue(){
  if(cmd_queue.empty()) return;


  TelemetryCommandQueueElement cmd = cmd_queue.front();
  rf95.send((uint8_t*)&cmd.command, sizeof(cmd.command));
  rf95.waitPacketSent();
}



SerialParser serial_parser(SerialInput, SerialError);

void setup() 
{
  while (!Serial);
  Serial.begin(9600);
  
  if (!rf95.init()) {
    Serial.println(json_init_failure);
    while (1);
  }

  Serial.println(json_init_success);

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(json_set_frequency_failure);
    while (1);
  }

  Serial.print(R"({"type": "freq_success", "frequency":)");
  Serial.print(RF95_FREQ);
  Serial.println("}");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}


void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    telemetry_data data{};
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      memcpy(&data, buf, sizeof(data));
      SerialPrintTelemetryData(data);

      if(!cmd_queue.empty()){
        auto & cmd = cmd_queue.front();
        if(cmd.command.id <= data.response_ID){
          if(cmd.command.command == CommandType::SET_FREQ){
            set_freq_local_bug_fix(cmd.command.freq);
            Serial.print(R"({"type": "freq_success", "frequency":)");
            Serial.print(cmd.command.freq);
            Serial.println("}");
          }
          cmd_queue.pop();
        } else {
          cmd.retry_count++;
          if(cmd.retry_count >= max_command_retries){
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