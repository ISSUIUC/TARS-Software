#ifndef SENSORS_CPP
#define SENSORS_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
//TODO remove
#include "main.cpp"

// #include "SparkFunLSM9DS1.h" //Low-G IMU Library
// #include "KX134-1211.h" //High-G IMU Library
// #include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
// #include "dataLog.h"
// #include "dataLog.cpp"
// #include "thresholds.h"
#include "pins.h"
#include "sensors.h"

static THD_FUNCTION(lowgIMU_THD, arg) {
  struct lowg_PNTR *pointer_struct = (struct lowg_PNTR *)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Low G IMU thread entrance");
    #endif

    chSysLock();
    pointer_struct->lowGimuPointer->readAccel();
    pointer_struct->lowGimuPointer->readGyro();
    pointer_struct->lowGimuPointer->readMag();
    chSysUnlock();

    pointer_struct->sensorDataPointer->timeStamp = chVTGetSystemTime();

    //acceleration in Gs
    pointer_struct->sensorDataPointer->ax = pointer_struct->lowGimuPointer->calcAccel(pointer_struct->lowGimuPointer->ax);
    pointer_struct->sensorDataPointer->ay = pointer_struct->lowGimuPointer->calcAccel(pointer_struct->lowGimuPointer->ay);
    pointer_struct->sensorDataPointer->az = pointer_struct->lowGimuPointer->calcAccel(pointer_struct->lowGimuPointer->az); //There was a minus here. We don't know why that did that
    //rotational speed in degrees per second
    pointer_struct->sensorDataPointer->gx = pointer_struct->lowGimuPointer->calcGyro(pointer_struct->lowGimuPointer->gx);
    pointer_struct->sensorDataPointer->gy = pointer_struct->lowGimuPointer->calcGyro(pointer_struct->lowGimuPointer->gy);
    pointer_struct->sensorDataPointer->gz = pointer_struct->lowGimuPointer->calcGyro(pointer_struct->lowGimuPointer->gz);
    //magnatometer data in gauss 
    pointer_struct->sensorDataPointer->mx = pointer_struct->lowGimuPointer->calcMag(pointer_struct->lowGimuPointer->mx);
    pointer_struct->sensorDataPointer->my = pointer_struct->lowGimuPointer->calcMag(pointer_struct->lowGimuPointer->my);
    pointer_struct->sensorDataPointer->mz = pointer_struct->lowGimuPointer->calcMag(pointer_struct->lowGimuPointer->mz);

    pointer_struct->sensorDataPointer->rocketState = *pointer_struct->rocketStatePointer;

    #ifdef LOWGIMU_DEBUG
      Serial.println("------------- LOW-G THREAD ---------------");
      Serial.print(pointer_struct->sensorDataPointer->ax);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->ay);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->az);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->gx);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->gy);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->gz);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->mx);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->my);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->mz);
      Serial.print(", ");
    #endif 

    // logData(&dataFile, &sensorData, rocketState);
    // add the data to the buffer here!
    if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        pointer_struct->dataloggerTHDVarsPointer->bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex);
    pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoHead] = *pointer_struct->sensorDataPointer;
    pointer_struct->dataloggerTHDVarsPointer->bufferErrors = 0;
    pointer_struct->dataloggerTHDVarsPointer->fifoHead = pointer_struct->dataloggerTHDVarsPointer->fifoHead < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer->fifoHead + 1 : 0;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoData);

    //!Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex);

    chThdSleepMilliseconds(6);
  }
}

static THD_FUNCTION(gps_THD, arg){
  struct gps_PNTR *pointer_struct = (struct gps_PNTR *)arg;
  while(true){
    
    #ifdef THREAD_DEBUG
      Serial.println("### GPS thread entrance");
    #endif

    pointer_struct->sensorDataPointer->timeStamp = chVTGetSystemTime();

    Serial.println("Running sys lock.");
    chSysLock();
    pointer_struct->GPSimuPointer->update_data();
    chSysUnlock();
    Serial.println("Running sys unlock.");

    //Have the availability to wait until a lock is aquired with gps.get_position_lock();
    pointer_struct->sensorDataPointer->latitude = pointer_struct->GPSimuPointer->get_latitude();
    pointer_struct->sensorDataPointer->longitude = pointer_struct->GPSimuPointer->get_longitude();
    pointer_struct->sensorDataPointer->altitude = pointer_struct->GPSimuPointer->get_altitude();
    pointer_struct->sensorDataPointer->posLock = pointer_struct->GPSimuPointer->get_position_lock();

    pointer_struct->sensorDataPointer->rocketState = *pointer_struct->rocketStatePointer;
    

    if(pointer_struct->sensorDataPointer->posLock == true){
      digitalWrite(LED_ORANGE, HIGH);
    }else{
      digitalWrite(LED_ORANGE, LOW);
    }

    #ifdef GPS_DEBUG
      bool position_lock = pointer_struct->GPSimuPointer->get_position_lock();
      if (position_lock) {
        Serial.println("POSITION LOCK!");
        Serial.println("GPS Data: ");
        Serial.print("Latitude: ");
        Serial.println(pointer_struct->sensorDataPointer->latitude);
        Serial.print("Longitude: ");
        Serial.println(pointer_struct->sensorDataPointer->longitude);
        Serial.print("Altitude: ");
        Serial.println(pointer_struct->sensorDataPointer->altitude);
      } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(pointer_struct->sensorDataPointer->latitude);
        Serial.print("Longitude: ");
        Serial.println(pointer_struct->sensorDataPointer->longitude);
        Serial.print("Altitude: ");
        Serial.println(pointer_struct->sensorDataPointer->altitude);
	      Serial.println("");
      }
    #endif

    // add the data to the buffer here!
    if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        pointer_struct->dataloggerTHDVarsPointer->bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    Serial.println("Locking GPS Mutex.");
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex);
    pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoHead] = *pointer_struct->sensorDataPointer;
    pointer_struct->dataloggerTHDVarsPointer->bufferErrors = 0;
    pointer_struct->dataloggerTHDVarsPointer->fifoHead = pointer_struct->dataloggerTHDVarsPointer->fifoHead < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer->fifoHead + 1 : 0;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoData);
 

    //!Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex);
    Serial.println("Unlocking GPS Mutex.");

    #ifdef THREAD_DEBUG
      Serial.println("### GPS thread exit");
    #endif

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}

static THD_FUNCTION(highgIMU_THD, arg){
  struct highg_PNTR *pointer_struct = (struct highg_PNTR *)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### High G IMU thread entrance");
    #endif

    

    chSysLock();
    pointer_struct->highGimuPointer->update_data();
    chSysUnlock();

    pointer_struct->sensorDataPointer->timeStamp = chVTGetSystemTime();

    //addition for highG IMU
    pointer_struct->sensorDataPointer->hg_ax = pointer_struct->highGimuPointer->get_x_gforce();
    pointer_struct->sensorDataPointer->hg_ay = pointer_struct->highGimuPointer->get_y_gforce();
    pointer_struct->sensorDataPointer->hg_az = pointer_struct->highGimuPointer->get_z_gforce();

    pointer_struct->sensorDataPointer->rocketState = *pointer_struct->rocketStatePointer;

    #ifdef HIGHGIMU_DEBUG
      Serial.println("------------- HIGH-G THREAD ---------------");
      //high g data
      Serial.print(pointer_struct->sensorDataPointer->hg_ax);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->hg_ay);
      Serial.print(", ");
      Serial.println(pointer_struct->sensorDataPointer->hg_az);
    #endif

    // add the data to the buffer here!
    if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        pointer_struct->dataloggerTHDVarsPointer->bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex);
    pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoHead] = *pointer_struct->sensorDataPointer;
    pointer_struct->dataloggerTHDVarsPointer->bufferErrors = 0;
    pointer_struct->dataloggerTHDVarsPointer->fifoHead = pointer_struct->dataloggerTHDVarsPointer->fifoHead < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer->fifoHead + 1 : 0;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoData);


    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex);

    chThdSleepMilliseconds(6);
  }
}
#endif