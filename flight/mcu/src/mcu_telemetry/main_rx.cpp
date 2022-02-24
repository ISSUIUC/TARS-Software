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


// Ensure to change depending on wiring
#define RFM95_CS 10
#define RFM95_RST 15
#define RFM95_EN 14
#define RFM95_INT 16
#define LED 13 // Blinks on receipt

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

#define DEFAULT_CMD 0
#define MAX_CMD_LEN 10

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//For reading from 
char incomingCmd[MAX_CMD_LEN];
char curByte;
short charIdx = 0;
short readySend = 0;

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
  char sign[8] = "KC1QJA";
};


enum class CommandType {
  SET_FREQ,
  SET_CALLSIGN,
  ABORT,
};
// Commands transmitted from ground station to rocket
struct telemetry_command {
  CommandType command;
  union {
    char callsign[8];
    int freq;
    bool do_abort;
  };
};


void SerialInput(const char * key, const char * value){
  telemetry_command t;
  
  if(strcmp(key, "ABORT") == 0){
    t.command = CommandType::ABORT;
    t.do_abort = true;
  } else if(strcmp(key, "FREQ") == 0) {
    t.command = CommandType::SET_FREQ;
    t.freq = atoi(value);
  } else if(strcmp(key, "CALLSIGN") == 0) {
    t.command = CommandType::SET_CALLSIGN;
    memcpy(t.callsign, value, 8);
  }

  rf95.send((uint8_t*)&t, sizeof(t));
  rf95.waitPacketSent();
  Serial.println("Sent a reply 2");
}

void SerialError(){

}

SerialParser serial_parser(SerialInput, SerialError);

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RFM95_EN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_EN, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

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
    telemetry_data data;
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      memcpy(&data, buf, sizeof(data));
      digitalWrite(LED, HIGH);
      // // This displays some of the data received 
      // Serial.println("Got: ");
      // Serial.print("GPS Lat ");
      // Serial.println(data.gps_lat);
      // Serial.print("GPS Long ");
      // Serial.println(data.gps_long);
      // Serial.print("GPS Alt ");
      // Serial.println(data.gps_alt);
      // Serial.print("Barometer Alt ");
      // Serial.println(data.barometer_alt);
      // Serial.print("KX IMU ax ");
      // Serial.println(data.KX_IMU_ax);
      // Serial.print("KX IMU ay ");
      // Serial.println(data.KX_IMU_ay);
      // Serial.print("KX IMU az ");
      // Serial.println(data.KX_IMU_az);
      // Serial.print("FSM state");
      // Serial.println(data.FSM_state);
      // Serial.print("RSSI: ");
      // Serial.println(rf95.lastRssi(), DEC);
      
    else
    {
      Serial.println("Receive failed");
    }
  }
  serial_parser.read();
}