// LoRa 9x_TX  (Rocket)
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <RH_RF95.h>

//Make sure to change these pinout depending on wiring
//Don't forget to change the ini file to build the correct main file
#define RFM95_CS 10
#define RFM95_RST 15
#define RFM95_INT 17
#define RFM95_EN 14

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

#define MAX_CMD_LEN 10

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


//For reading from 
char incomingCmd[MAX_CMD_LEN];

// input value for sine function
double dummy_input = 0; 



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

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RFM95_EN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_EN, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Telemetry Test");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Radio Initialization Failed");
    while (1);
  }
  Serial.println("Radio Initialized");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency Failed");
    while (1);
  }
  Serial.print("Frequency set to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission
int last_command_id = -1;
char callsign[8] = "NO SIGN";

struct {
  bool should_change{};
  int new_freq{};
} freq_status;

void handle_command(const telemetry_command & cmd){
/* Check if lasted command ID matched current command ID */
      if(last_command_id == cmd.cmd_id){
        return;
      }
      last_command_id = cmd.cmd_id;
      if (cmd.command == SET_FREQ) {
        freq_status.should_change = true;
        freq_status.new_freq = cmd.freq;
      } 

      if (cmd.command == SET_CALLSIGN) {
        memcpy(callsign, cmd.callsign, sizeof(cmd.callsign));
      }
      
      Serial.println("Got Commands:");
      Serial.print("Call Sign: ");
      Serial.println(cmd.callsign);

      Serial.print("Abort? ");
      Serial.println(cmd.do_abort);
      Serial.print("Frequency: ");
      Serial.println(cmd.freq);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);  
}

void loop()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  
  telemetry_data d{};

  // Looping input value from 0 to 2pi over and over 
  if (dummy_input > 628) {
    dummy_input = 0;
  } else {
    dummy_input+=30;
  }

  // Computing sine value
  double sin_value = sin(dummy_input/100);
  double cos_value = cos(dummy_input/100);
  double tan_value = tan(dummy_input/100);


  // Setting each sensor value to the current sine value
  d.gps_lat=sin_value;
  d.gps_long=cos_value;
  d.gps_alt=-1 * dummy_input/100;
  d.barometer_alt=sin_value;
  d.KX_IMU_ax=cos_value;
  d.KX_IMU_ay=sin_value;
  d.KX_IMU_az=dummy_input/100;
  d.H3L_IMU_ax=sin_value;
  d.H3L_IMU_ay=cos_value;
  d.H3L_IMU_az=sin_value+cos_value;
  d.LSM_IMU_ax=cos_value;    
  d.LSM_IMU_ay=sin_value;
  d.LSM_IMU_az=((dummy_input/100)*(dummy_input/100))/2;
  d.LSM_IMU_gx=tan_value;    
  d.LSM_IMU_gy=sin_value;
  d.LSM_IMU_gz=cos_value;
  d.LSM_IMU_mx=sin_value;
  d.LSM_IMU_my=sin_value;
  d.LSM_IMU_mz=sin_value;

  d.rssi = rf95.lastRssi();

  d.response_ID = last_command_id;
  memcpy(d.sign, callsign, sizeof(callsign));
  
  Serial.println("Sending sample sensor data..."); delay(10);
  rf95.send((uint8_t *)&d, sizeof(d));

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();

  //change the freqencey after we acknowledge
  if(freq_status.should_change){
    rf95.setFrequency(freq_status.new_freq);
    freq_status.should_change = false;
  }
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
                                          //test without delay
  Serial.println("Waiting for reply..."); //delay(10);
  if (rf95.available())
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      telemetry_command received;
      memcpy(&received, buf, sizeof(received));
      
      handle_command(received);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  delay(100);
}