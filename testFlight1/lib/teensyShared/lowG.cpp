#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
//TODO remove
#include "main.cpp"

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"

static THD_FUNCTION(lowgIMU_THD, arg) {
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Low G IMU thread entrance");
    #endif

    chSysLock();
    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();
    chSysUnlock();
    Serial.println("Sys unlocked.");

    lowgSensorData.timeStamp = chVTGetSystemTime();

    //acceleration in Gs
    lowgSensorData.ax = lowGimu.calcAccel(lowGimu.ax);
    lowgSensorData.ay = lowGimu.calcAccel(lowGimu.ay);
    lowgSensorData.az = lowGimu.calcAccel(lowGimu.az); //There was a minus here. We don't know why that did that
    //rotational speed in degrees per second
    lowgSensorData.gx = lowGimu.calcGyro(lowGimu.gx);
    lowgSensorData.gy = lowGimu.calcGyro(lowGimu.gy);
    lowgSensorData.gz = lowGimu.calcGyro(lowGimu.gz);
    //magnatometer data in gauss 
    lowgSensorData.mx = lowGimu.calcMag(lowGimu.mx);
    lowgSensorData.my = lowGimu.calcMag(lowGimu.my);
    lowgSensorData.mz = lowGimu.calcMag(lowGimu.mz);

    lowgSensorData.rocketState = rocketState;

    #ifdef LOWGIMU_DEBUG
      Serial.println("------------- LOW-G THREAD ---------------");
      Serial.print(lowgSensorData.ax);
      Serial.print(", ");
      Serial.print(lowgSensorData.ay);
      Serial.print(", ");
      Serial.print(lowgSensorData.az);
      Serial.print(", ");
      Serial.print(lowgSensorData.gx);
      Serial.print(", ");
      Serial.print(lowgSensorData.gy);
      Serial.print(", ");
      Serial.print(lowgSensorData.gz);
      Serial.print(", ");
      Serial.print(lowgSensorData.mx);
      Serial.print(", ");
      Serial.print(lowgSensorData.my);
      Serial.print(", ");
      Serial.print(lowgSensorData.mz);
      Serial.print(", ");
    #endif 

    // logData(&dataFile, &sensorData, rocketState);
    // add the data to the buffer here!
    if (chSemWaitTimeout(&lowg_datalogger_THD_vars.fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        lowg_datalogger_THD_vars.bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&lowg_datalogger_THD_vars.dataMutex);
    lowg_datalogger_THD_vars.fifoArray[lowg_datalogger_THD_vars.fifoHead] = lowgSensorData;
    lowg_datalogger_THD_vars.bufferErrors = 0;
    lowg_datalogger_THD_vars.fifoHead = lowg_datalogger_THD_vars.fifoHead < (FIFO_SIZE - 1) ? lowg_datalogger_THD_vars.fifoHead + 1 : 0;
    chSemSignal(&lowg_datalogger_THD_vars.fifoData);

    //!Unlocking &dataMutex
    chMtxUnlock(&lowg_datalogger_THD_vars.dataMutex);

    chThdSleepMilliseconds(6);
  }
}
