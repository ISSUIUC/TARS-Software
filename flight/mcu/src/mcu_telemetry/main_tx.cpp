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


// Data transmitted from rocket to ground station
struct telemetry_data {

  // GPS Data
  double gps_lat = 41.4804;
  double gps_long = -89.501;
  double gps_alt = 2362.07;

  // Barometer Data
  double barometer_alt = 2000.0;
  
  // Rocket Software FSM State
  int FSM_state = 2;

  // KX134 (HIGH_G) IMU Data
  double HIGHG_IMU_ax = 0.4209;
  double HIGHG_IMU_ay = -0.733464;
  double HIGHG_IMU_az = -0.5124;
  double HIGHG_IMU_gx = 26.88;
  double HIGHG_IMU_gy = -553.84;
  double HIGHG_IMU_gz = 0.49;
  double HIGHG_IMU_mx = 0.4018;
  double HIGHG_IMU_my = -0.36078;
  double HIGHG_IMU_mz = 0.16828;

  // LSM9DS1 (LOW_G) IMU Data
  double LOWG_IMU_ax = 0.4209;
  double LOWG_IMU_ay = -0.733464;
  double LOWG_IMU_az = -0.5124;
  double LOWG_IMU_gx = 26.88;
  double LOWG_IMU_gy = -553.84;
  double LOWG_IMU_gz = 0.49;
  double LOWG_IMU_mx = 0.4018;
  double LOWG_IMU_my = -0.36078;
  double LOWG_IMU_mz = 0.16828;

  // Callsign
  char sign[];

};

// Commands transmitted from ground station to rocket
struct telemetry_command {
  int command;
  char change_callsign[];
  float change_frequency;
>>>>>>> 3ada75a0bc86c38ca08151d13a50256abe9191f4
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

void loop()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  
  telemetry_data d;
  // char radiopacket[20] = "Hey bestie #      ";
  // itoa(packetnum++, radiopacket+13, 10);
  // Serial.print("Sending "); Serial.println(radiopacket);
  // radiopacket[19] = 0;
  
  Serial.println("Sending sample sensor data..."); delay(10);
  rf95.send((uint8_t *)&d, sizeof(d));

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
                                          //test without delay
  Serial.println("Waiting for reply..."); //delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      memcpy(incomingCmd, buf, sizeof(incomingCmd));
      Serial.print("Got reply: ");

      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
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
  delay(1000);
}