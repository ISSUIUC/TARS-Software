#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "thresholds.h"
#include "pins.h"

//#define THREAD_DEBUG
#define IMU_DEBUG
// #define GPS_DEBUG

//changed name to account for both high & lowG (logGData)
dataStruct_t sensorData;

FSM_State rocketState = STATE_INIT;

File dataFile;

KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();

static THD_WORKING_AREA(sensor_WA, 512);

static THD_FUNCTION(sensor_THD, arg){
  (void)arg;

  #ifdef THREAD_DEBUG
    Serial.println("### data thread entrance");
  #endif

  while(true){
    //!locking data from sensorData struct

    chSysLock();
    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();
    highGimu.update_data();
    chSysLock();

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
    //addition for highG IMU
    sensorData.hg_ax = highGimu.get_x_gforce();
    sensorData.hg_ay = highGimu.get_y_gforce();
    sensorData.hg_az = highGimu.get_z_gforce();
    sensorData.timeStamp = chVTGetSystemTime();

    #ifdef IMU_DEBUG
      Serial.print(sensorData.ax);
      Serial.print(" ");
      Serial.print(sensorData.ay);
      Serial.print(" ");
      Serial.print(sensorData.az);
      Serial.print(" ");
      Serial.print(sensorData.hg_ax);
      Serial.print(" ");
      Serial.print(sensorData.hg_ay);
      Serial.print(" ");
      Serial.print(sensorData.hg_az);
      Serial.print(" \r");
    #endif

    chSysLock();
    gps.update_data();
    chSysLock();

    //Have the availability to wait until a lock is aquired with gps.get_position_lock();
    sensorData.latitude = gps.get_latitude();
    sensorData.longitude = gps.get_longitude();
    sensorData.altitude = gps.get_altitude();

    #ifdef GPS_DEBUG
        Serial.println("------------- GPS ---------------");
      bool position_lock = gps.get_position_lock();
      if (position_lock) {
        Serial.println("POSITION LOCK!");
      } else {
        Serial.println("No lock found");
      }
      Serial.println("GPS Data: ");
      Serial.print("Latitude: ");
      Serial.println(sensorData.latitude);
      Serial.print("Longitude: ");
      Serial.println(sensorData.longitude);
      Serial.print("Altitude: ");
      Serial.println(sensorData.altitude);
      Serial.println("");
    #endif

    // chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz

    logData(&dataFile, &sensorData, rocketState);

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}

void chSetup(){
  chThdCreateStatic(sensor_WA, sizeof(sensor_WA), NORMALPRIO, sensor_THD, NULL);
  while(true);
}

void setup() {

  Serial.begin(115200);
  while (!Serial) {}

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);

  digitalWrite(LED_BLUE, HIGH);

  //lowGimu setup
  if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    digitalWrite(LED_RED, HIGH);
    Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
    while (true);
  }

  lowGimu.setAccelScale(16);

  //GPS Setup
 	gps.beginSPI(ZOEM8Q0_CS);

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
