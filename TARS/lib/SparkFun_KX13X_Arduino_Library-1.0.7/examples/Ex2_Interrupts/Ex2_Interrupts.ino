/*********************************************************
Author: Elias Santistevan @ SparkFun Electronics
Date: 5/2021
Library repository can be found here:
https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library

This example demonstrates the use of hardware interrupts for synchronizing
acceleromter data using I2C.

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
*******************************************************/
#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"

QwiicKX132 kxAccel;
//QwiicKX134 kxAccel; // Uncomment this if using the KX134 - check your board
                      //if unsure.  

outputData myData; // This will hold the accelerometer's output. 
int dataReadyPin = 3; 

void setup() {

  pinMode(dataReadyPin, INPUT);

  while(!Serial){
    delay(50);
  }

  Serial.begin(115200);
  Serial.println("Welcome.");

  Wire.begin();
  if( !kxAccel.begin() ){
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while(1);
  }
  else
    Serial.println("Ready.");
    


  if( !kxAccel.initialize(INT_SETTINGS)){ // Load preset interrupt settings.
    Serial.println("Could not initialize the chip. Freezing.");
    while(1);
  }
  else
    Serial.println("Initialized...");

  // kxAccel.setRange(KX132_RANGE16G);
  // kxAccel.setRange(KX134_RANGE32G); // For a larger range uncomment
}

void loop() {

  if( digitalRead(dataReadyPin) == HIGH ){ // Wait for new data to be ready.

    myData = kxAccel.getAccelData();
    Serial.print("X: ");
    Serial.print(myData.xData, 4);
    Serial.print("g ");
    Serial.print(" Y: ");
    Serial.print(myData.yData, 4);
    Serial.print("g ");
    Serial.print(" Z: ");
    Serial.print(myData.zData, 4);
    Serial.println("g ");
    
     //kxAccel.clearInterrupt();// Because the data is being read in "burst"
     //mode, meaning that all the acceleration data is being read at once, we don't
     //need to clear the interrupt.
  }
  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
}
