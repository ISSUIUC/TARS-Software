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

//!Creating mutex to prevent overlapping reads from play_THD and THD_FUNCTION
//!for reading sensorData struct
static MUTEX_DECL(dataMutex);

#define PLAY_DEBUG
// #define THREAD_DEBUG
#define IMU_DEBUG
//#define GPS_DEBUG

//changed name to account for both high & lowG (logGData)
dataStruct_t sensorData;

FSM_State rocketState;

File dataFile;

KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();


static THD_WORKING_AREA(sensor_WA, 256);
static THD_WORKING_AREA(play_WA, 256);

static THD_FUNCTION(sensor_THD, arg){
  (void)arg;



  #ifdef THREAD_DEBUG
    Serial.println("### data thread entrance");
  #endif
  
  while(true){
    //!locking data from sensorData struct
    chMtxLock(&dataMutex);

    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();

    highGimu.update_data();

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
      Serial.println("------------- IMU THREAD ---------------")
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
      //high g data
      Serial.print(sensorData.hg_ax);
      Serial.print(", ");
      Serial.print(sensorData.hg_ay);
      Serial.print(", ");
      Serial.println(sensorData.hg_az);
    #endif

    #ifdef GPS_DEBUG
      while (!gps.update_data()) {
        Serial.println(".");
        chThdSleepMilliseconds(500);
      }
      Serial.println("");
      bool position_lock = gps.get_position_lock();
      if (position_lock) {
        sensorData.latitude = gps.get_latitude();
        sensorData.longitude = gps.get_longitude();
        sensorData.altitude = gps.get_altitude();
        Serial.println("GPS Data in sensorData struct: ");
        Serial.print("Latitude: ");
        Serial.println(sensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(sensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(sensorData.altitude);
      } else {
        Serial.println("Searching...");
        sensorData.latitude = 0;
        sensorData.longitude = 0;
        sensorData.altitude = 0;
      }
      chThdSleepMilliseconds(500);
    #endif
    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz

    //!Unlocking &dataMutex
    chMtxUnlock(&dataMutex);


    logData(&dataFile, &sensorData, rocketState);

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}


static THD_FUNCTION(play_THD, arg){
  (void)arg;

  while(true){
    //!locking mutex to get data from sensorData struct
    chMtxLock(&dataMutex);

    #ifdef PLAY_DEBUG
      //! taking the data from sensorData and multiplying by 69
      //!nice
      Serial.println("------------ Play thread -------------")
      Serial.print(sensorData.ax * 69);
      Serial.print(", ");
      Serial.print(sensorData.ay * 69);
      Serial.print(", ");
      Serial.print(sensorData.az * 69);
      Serial.print(", ");
      Serial.print(sensorData.gx * 69);
      Serial.print(", ");
      Serial.print(sensorData.gy * 69);
      Serial.print(", ");
      Serial.print(sensorData.gz * 69);
      Serial.print(", ");
      Serial.print(sensorData.mx * 69);
      Serial.print(", ");
      Serial.print(sensorData.my * 69);
      Serial.print(", ");
      Serial.print(sensorData.mz * 69);
      Serial.print(", ");
      //high g data
      Serial.print(sensorData.hg_ax * 69);
      Serial.print(", ");
      Serial.print(sensorData.hg_ay * 69);
      Serial.print(", ");
      Serial.println(sensorData.hg_az * 69);
    #endif

    //!unlocking &dataMutex mutex
    chMtxUnlock(&dataMutex);
  }

}


void chSetup(){
  //added play_THD for creation
  chThdCreateStatic(sensor_WA, sizeof(sensor_WA), NORMALPRIO, sensor_THD, NULL);
  chThdCreateStatic(play_WA, sizeof(play_WA), NORMALPRIO, play_THD, NULL);
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