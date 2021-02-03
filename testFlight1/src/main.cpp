#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "thresholds.h"
#include "pins.h"

#define THREAD_DEBUG
#define SENSOR_DEBUG

//!possibly change name to account for both high & lowG (logGData)
dataStruct_t sensorData;

FSM_State rocketState;

File dataFile;

KX134 highGimu;
LSM9DS1 lowGimu;

static THD_WORKING_AREA(imu_WA, 32);
static THD_WORKING_AREA(gps_WA, 32);

static THD_FUNCTION(imu_THD, arg){
  (void)arg;

  #ifdef THREAD_DEBUG
    Serial.println("### data thread entrance");
  #endif
  
  while(true){
    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();

    //acceleration in Gs
    sensorData.ax = lowGimu.calcAccel(lowGimu.ax);
    sensorData.ay = lowGimu.calcAccel(lowGimu.ay);
    sensorData.az = lowGimu.calcAccel(lowGimu.az); //There was a minus here. We don't know why that did that
    //rotational speed in degrees per second
    sensorData.gx = lowGimu.calcGyro(lowGimu.gx);
    sensorData.gy = lowGimu.calcGyro(lowGimu.gy);
    sensorData.gz = lowGimu.calcGyro(lowGimu.gz);
    //magnatometer data in gauss 
    sensorData.mx = lowGimu.calcMag(lowGimu.mx);
    sensorData.my = lowGimu.calcMag(lowGimu.my);
    sensorData.mz = lowGimu.calcMag(lowGimu.mz);
    //!addition for highG IMU
    sensorData.hg_ax = highGimu.get_x_gforce();
    sensorData.hg_ay = highGimu.get_y_gforce();
    sensorData.hg_az = highGimu.get_z_gforce();
    sensorData.timeStamp = chVTGetSystemTime();

    #ifdef SENSOR_DEBUG
      Serial.print(sensorData.ax);
      Serial.print(", ");
      Serial.print(sensorData.ay);
      Serial.print(", ");
      Serial.print(sensorData.az);
      Serial.print(", ");
      Serial.print(sensorData.gx);
      Serial.print(", ");
      Serial.print(sensorData.gy);
      Serial.print(", ");
      Serial.print(sensorData.gz);
      Serial.print(", ");
      Serial.print(sensorData.mx);
      Serial.print(", ");
      Serial.print(sensorData.my);
      Serial.print(", ");
      Serial.print(sensorData.mz);
      Serial.print(", ");
      //!high g data
      Serial.print(sensorData.hg_ax);
      Serial.print(", ");
      Serial.print(sensorData.hg_ay);
      Serial.print(", ");
      Serial.println(sensorData.hg_az);
    #endif



    logData(&dataFile, &sensorData, rocketState);

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}

static THD_FUNCTION(gps_THD, arg){

}

void chSetup(){
  chThdCreateStatic(imu_WA, sizeof(imu_WA), NORMALPRIO, imu_THD, NULL);
  chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD, NULL);
  while(true);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  rocketState = STATE_IDLE;

  //lowGimu setup
  if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    digitalWrite(LED_RED, HIGH);
    Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
    while (1);
  }

  //SD Card Setup
  if(SD.begin(BUILTIN_SDCARD)){
    init_dataLog(&dataFile);
  }
  else {
    digitalWrite(LED_RED, HIGH);
    Serial.println("SD Begin Failed. Stalling Program");
    while(true);
  }

  Serial.println("Starting ChibiOS");
  chBegin(chSetup);
  while(true);

  
}

void loop() {
  // not used
}