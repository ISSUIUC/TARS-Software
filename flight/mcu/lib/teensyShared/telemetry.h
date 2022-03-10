#include <SPI.h>
#include <RH_RF95.h>
#include <ChRt.h>

//Make sure to change these pinout depending on wiring
//Don't forget to change the ini file to build the correct main file
#define RFM95_CS 10
#define RFM95_RST 15
#define RFM95_INT 17
#define RFM95_EN 14

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

#define MAX_CMD_LEN 10
// Change to 434.0 or other frequency, must match RX's freq!

// Data transmitted from rocket to ground station
struct telemetry_data {
  double gps_lat;
  double gps_long;
  double gps_alt;
  double barometer_alt;
  // KX134 (highg) IMU DATA
  double KX_IMU_ax;   // acceleration (in G's)
  double KX_IMU_ay;
  double KX_IMU_az;
  // H3LIS331DL (highg) IMU DATA
  double H3L_IMU_ax;
  double H3L_IMU_ay;
  double H3L_IMU_az;
  // LSM9DS1 (lowg) IMU DATA
  double LSM_IMU_ax;    // acceleration (in G's)
  double LSM_IMU_ay;
  double LSM_IMU_az;
  double LSM_IMU_gx;    // Gyro data (in degrees/sec)
  double LSM_IMU_gy;
  double LSM_IMU_gz;
  double LSM_IMU_mx;
  double LSM_IMU_my;
  double LSM_IMU_mz;
  
  int FSM_state;
  char sign[8] = "HITHERE";
  int rssi;
  double battery_voltage;
  int response_ID;
};

// Commands transmitted from ground station to rocket
enum CommandType {
  SET_FREQ,
  SET_CALLSIGN,
  ABORT,
  EMPTY
};

struct telemetry_command {
  CommandType command;
  int cmd_id;
  union {
    char callsign[8];
    int freq;
    bool do_abort;
  };
};

class Telemetry {
    public:
        Telemetry();
        void receive();
        void transmit();
    private:
        int packetnum;
        telemetry_data d;
        RH_RF95 rf95;
};