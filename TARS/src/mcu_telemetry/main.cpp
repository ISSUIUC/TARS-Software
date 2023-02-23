/*
  Reading Position, Velocity and Time (PVT) via UBX binary commands

  This example shows how to query a u-blox module for its position, velocity and time (PVT) data.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

  Note: Lat/Lon are large numbers because they are * 10^7. To convert lat/lon
  to something google maps understands simply divide the numbers by 10,000,000.

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and your microcontroller board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

SFE_UBLOX_GNSS myGNSS; // SFE_UBLOX_GNSS uses I2C. For Serial or SPI, see Example2 and Example3

void setup()
{
  Serial.begin(115200);
  delay(1000); 
  Serial.println("SparkFun u-blox Example");
  // Wire.setPins(2, 1);
  Wire.begin(2, 1); // Start I2C
  

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
    delay (1000);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR
}

void loop()
{
  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (myGNSS.getPVT() == true)
  {
    int32_t latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
}

// #include <Arduino.h>
// #include <FreeRTOS.h>
// TaskHandle_t Task1;
// TaskHandle_t Task2;
// // void Task1code();
// // void Task2code();
// // LED pins
// void Task1code(void* param){
//   Serial.print("Task1 running on core ");
//   Serial.println(xPortGetCoreID());
// }
// //Task2code: blinks an LED every 700 ms
// void Task2code(void* param){
//   Serial.print("Task2 running on core ");
//   Serial.println(xPortGetCoreID());
// }

// void setup() {
//   Serial.begin(115200);
//   //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
//   xTaskCreatePinnedToCore(
//                     Task1code,   /* Task function. */
//                     "Task1",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &Task1,      /* Task handle to keep track of created task */
//                     0);          /* pin task to core 0 */
//   delay(500);
//   //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
//   xTaskCreatePinnedToCore(
//                     Task2code,   /* Task function. */
//                     "Task2",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &Task2,      /* Task handle to keep track of created task */
//                     1);          /* pin task to core 1 */
//     delay(500);
// }
// //Task1code: blinks an LED every 1000 ms

// void loop() {
// }