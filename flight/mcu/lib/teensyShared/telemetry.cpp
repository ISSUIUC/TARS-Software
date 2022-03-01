#include <telemetry.h>


Telemetry::Telemetry(): rf95(RFM95_CS, RFM95_INT) {

    pinMode(RFM95_RST, OUTPUT);
    pinMode(RFM95_EN, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    digitalWrite(RFM95_EN, HIGH);
    // while (!Serial);
    // Serial.begin(9600);

    delay(100);

    // Serial.println("Telemetry Test");

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        // Serial.println("Radio Initialization Failed");
        while (1);
    }
    // Serial.println("Radio Initialized");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        // Serial.println("setFrequency Failed");
        while (1);
    }
    // Serial.print("Frequency set to: "); Serial.println(RF95_FREQ);
    
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
}

void Telemetry::transmit() {
  d.rssi = rf95.lastRssi();
  // char radiopacket[20] = "Hey bestie #      ";
  // itoa(packetnum++, radiopacket+13, 10);
  // Serial.print("Sending "); Serial.println(radiopacket);
  // radiopacket[19] = 0;
  
  // Serial.println("Sending sample sensor data..."); delay(10);
  rf95.send((uint8_t *)&d, sizeof(d));

  // Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
//   Serial.print("sent, RSSI:"); Serial.println(d.rssi);
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
                                          //test without delay
  // Serial.println("Waiting for reply..."); //delay(10);
  if (rf95.available())
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      telemetry_command received;
      memcpy(&received, buf, sizeof(received));
      if (received.command == SET_FREQ) {
        rf95.setFrequency(received.freq);
      } 

      if (received.command == SET_CALLSIGN) {
        memcpy(d.sign, received.callsign, sizeof(received.callsign));
      }
      
    //   Serial.println("Got Commands:");
    //   Serial.print("Call Sign: ");
    //   Serial.println(received.callsign);

    //   Serial.print("Abort? ");
    //   Serial.println(received.do_abort);
    //   Serial.print("Frequency: ");
    //   Serial.println(received.freq);
    //   Serial.print("RSSI: ");
    //   Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
    //   Serial.println("Receive failed");
    }
  }
  else
  {
    // Serial.println("No reply, is there a listener around?");
  }
  delay(1000);
    
}