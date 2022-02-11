#include <telemetry.h>


Telemetry::Telemetry() {
    int packetnum = 0;
    // RH_RF95 rf95(RFM95_CS, RFM95_INT);
}

void Telemetry::receive() {
    //test with demo rx code
}

void Telemetry::transmit(const RH_RF95& rf95) {
    //test with demo tx code
    Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
        char radiopacket[40] = "Hey bestie#      ";
        itoa(packetnum++, radiopacket+13, 10);
        Serial.print("Sending "); Serial.println(radiopacket);
        
        Serial.println("Sending..."); //delay(10);
        // chThdSleepMilliseconds(10);
        rf95.send((uint8_t *)radiopacket, 40);

        Serial.println("Waiting for packet to complete..."); //delay(10);
        // delay(10);
        // chThdSleepMilliseconds(10);
        rf95.waitPacketSent();
        // Now wait for a reply
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
                                                //test without delay
        Serial.println("Waiting for reply...");  
        // delay(10);
        // chThdSleepMilliseconds(10);
        if (rf95.waitAvailableTimeout(1000))
        { 
            // Should be a reply message for us now   
            if (rf95.recv(buf, &len))
        {
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
        // chThdSleepMilliseconds(800);
}

