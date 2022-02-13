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


// Ensure to change depending on wiring
#define RFM95_CS 10
#define RFM95_RST 15
#define RFM95_EN 14
#define RFM95_INT 16
#define LED 13 // Blinks on receipt

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct TELEM_DATA{
  short data1;
  short data2;
  short data3;
  short data4;
  short data5;
  short data6;
  short data7;
  short data8;
  short data9;
  short data10;
  short data11;
  short data12;
  short data13;
  short data14;
  short data15;
  short data16;

};

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
    TELEM_DATA data;
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      memcpy(&data, buf, sizeof(data));
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println(data.data1);
      Serial.println(data.data2);
      Serial.println(data.data3);
      Serial.println(data.data4);
      Serial.println(data.data5);
      Serial.println(data.data6);
      Serial.println(data.data7);
      Serial.println(data.data8);
      Serial.println(data.data9);
      Serial.println(data.data10);
      Serial.println(data.data11);
      Serial.println(data.data12);
      Serial.println(data.data13);
      Serial.println(data.data14);
      Serial.println(data.data15);
      Serial.println(data.data16);
      //Serial.println(data.data2);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Send a reply
      uint8_t data[] = "And hello back to you"; // This is currently not being received by the transmitter. 
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}