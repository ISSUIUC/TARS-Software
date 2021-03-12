#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.h" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "thresholds.h"
#include "pins.h"

//!Creating mutex to prevent overlapping reads from play_THD and THD_FUNCTION
//!for reading sensorData struct
static MUTEX_DECL(lowg_dataMutex);
static MUTEX_DECL(highg_dataMutex);
static MUTEX_DECL(gps_dataMutex);

//Variables for creating a ring buffer of dataStruct_t's:
#define FIFO_SIZE 3000
SEMAPHORE_DECL(lowg_fifoData, 0);
SEMAPHORE_DECL(lowg_fifoSpace, FIFO_SIZE);
static lowg_dataStruct_t lowg_fifoArray[FIFO_SIZE];
uint16_t lowg_fifoHead = 0;
uint16_t lowg_fifoTail = 0;
uint16_t lowg_bufferErrors = 0;

SEMAPHORE_DECL(highg_fifoData, 0);
SEMAPHORE_DECL(highg_fifoSpace, FIFO_SIZE);
static highg_dataStruct_t highg_fifoArray[FIFO_SIZE];
uint16_t highg_fifoHead = 0;
uint16_t highg_fifoTail = 0;
uint16_t highg_bufferErrors = 0;

SEMAPHORE_DECL(gps_fifoData, 0);
SEMAPHORE_DECL(gps_fifoSpace, FIFO_SIZE);
static gps_dataStruct_t gps_fifoArray[FIFO_SIZE];
uint16_t gps_fifoHead = 0;
uint16_t gps_fifoTail = 0;
uint16_t gps_bufferErrors = 0;

//#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG

//changed name to account for both high & lowG (logGData)
gps_dataStruct_t gpsSensorData;
lowg_dataStruct_t lowgSensorData;
highg_dataStruct_t highgSensorData;

FSM_State rocketState = STATE_INIT;
fsm_struct rocketTimers;

File lowg_dataFile;
File highg_dataFile;
File gps_dataFile;

KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();


static THD_WORKING_AREA(gps_WA, 512);
static THD_WORKING_AREA(rocket_FSM_WA, 512);
static THD_WORKING_AREA(lowgIMU_WA, 512);
static THD_WORKING_AREA(highgIMU_WA, 512);
static THD_WORKING_AREA(lowg_dataLogger_WA, 512);
static THD_WORKING_AREA(highg_dataLogger_WA, 512);
static THD_WORKING_AREA(gps_dataLogger_WA, 512);
/*
static THD_FUNCTION(dataLogger_THD, arg){
  (void)arg;
  //struct dataLogger_args *myarg = arg;
  while(true){
    #ifdef THREAD_DEBUG
      Serial.println("### Data Logging thread entrance");
    #endif
    chSemWait(myarg->fifoData);
    chMtxLock(&dataMutex);

    lowg_dataStruct_t data = fifoArray[fifoTail];
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
    chSemSignal(myarg->fifoSpace);
    chMtxUnlock(&dataMutex);
    
    logData(myarg->dataFile, &data, rocketState);
  }
}
*/

/* static THD_FUNCTION(lowg_dataLogger_THD, arg){
  (void)arg;
  while(true){
    #ifdef THREAD_DEBUG
      Serial.println("### Low g Data Logging thread entrance");
    #endif
    chSemWait(&lowg_fifoData);
    chMtxLock(&lowg_dataMutex);

    lowg_dataStruct_t data = lowg_fifoArray[lowg_fifoTail];
    lowg_fifoTail = lowg_fifoTail < (FIFO_SIZE - 1) ? lowg_fifoTail + 1 : 0;
    chSemSignal(&lowg_fifoSpace);
    chMtxUnlock(&lowg_dataMutex);
    
    logData(&lowg_dataFile, &data, rocketState);

    chThdSleepMilliseconds(6);
  }
} */

static THD_FUNCTION(highg_dataLogger_THD, arg){
  (void)arg;
  while(true){
    #ifdef THREAD_DEBUG
      Serial.println("### High g Data Logging thread entrance");
    #endif
    chSemWait(&highg_fifoData);
    chMtxLock(&highg_dataMutex);

    highg_dataStruct_t data = highg_fifoArray[highg_fifoTail];
    highg_fifoTail = highg_fifoTail < (FIFO_SIZE - 1) ? highg_fifoTail + 1 : 0;
    chSemSignal(&highg_fifoSpace);
    chMtxUnlock(&highg_dataMutex);
    
    logData(&highg_dataFile, &data, rocketState);

    chThdSleepMilliseconds(6);
  }
}

static THD_FUNCTION(gps_dataLogger_THD, arg){
  (void)arg;
  while(true){
    #ifdef THREAD_DEBUG
      Serial.println("### gps Data Logging thread entrance");
    #endif
    chSemWait(&gps_fifoData);
    chMtxLock(&gps_dataMutex);

    gps_dataStruct_t data = gps_fifoArray[gps_fifoTail];
    gps_fifoTail = gps_fifoTail < (FIFO_SIZE - 1) ? gps_fifoTail + 1 : 0;
    chSemSignal(&gps_fifoSpace);
    chMtxUnlock(&gps_dataMutex);
    
    logData(&gps_dataFile, &data, rocketState);

    chThdSleepMilliseconds(6);
  }
}

static THD_FUNCTION(gps_THD, arg){
  (void)arg;
  while(true){
    
    #ifdef THREAD_DEBUG
      Serial.println("### GPS thread entrance");
    #endif

    gpsSensorData.timeStamp = chVTGetSystemTime();

    chSysLock();
    gps.update_data();
    chSysUnlock();

    //Have the availability to wait until a lock is aquired with gps.get_position_lock();
    gpsSensorData.latitude = gps.get_latitude();
    gpsSensorData.longitude = gps.get_longitude();
    gpsSensorData.altitude = gps.get_altitude();
    gpsSensorData.posLock = gps.get_position_lock();
    

    if(gpsSensorData.posLock == true){
      digitalWrite(LED_ORANGE, HIGH);
    }else{
      digitalWrite(LED_ORANGE, LOW);
    }

    #ifdef GPS_DEBUG
      bool position_lock = gps.get_position_lock();
      if (position_lock) {
        Serial.println("POSITION LOCK!");
        Serial.println("GPS Data: ");
        Serial.print("Latitude: ");
        Serial.println(gpsSensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(gpsSensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(gpsSensorData.altitude);
      } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(gpsSensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(gpsSensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(gpsSensorData.altitude);
	      Serial.println("");
      }
    #endif

    // add the data to the buffer here!
    if (chSemWaitTimeout(&gps_fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        gps_bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&gps_dataMutex);
    gps_fifoArray[gps_fifoHead] = gpsSensorData;
    gps_bufferErrors = 0;
    gps_fifoHead = gps_fifoHead < (FIFO_SIZE - 1) ? gps_fifoHead + 1 : 0;
    chSemSignal(&gps_fifoData);
 

    //!Unlocking &dataMutex
    chMtxUnlock(&gps_dataMutex);  

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}


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
    if (chSemWaitTimeout(&lowg_fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        lowg_bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&lowg_dataMutex);
    lowg_fifoArray[lowg_fifoHead] = lowgSensorData;
    lowg_bufferErrors = 0;
    lowg_fifoHead = lowg_fifoHead < (FIFO_SIZE - 1) ? lowg_fifoHead + 1 : 0;
    chSemSignal(&lowg_fifoData);

    //!Unlocking &dataMutex
    chMtxUnlock(&lowg_dataMutex);

    chThdSleepMilliseconds(6);
  }
}


static THD_FUNCTION(highgIMU_THD, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### High G IMU thread entrance");
    #endif

    

    chSysLock();
    highGimu.update_data();
    chSysUnlock();

    highgSensorData.timeStamp = chVTGetSystemTime();

    //addition for highG IMU
    highgSensorData.hg_ax = highGimu.get_x_gforce();
    highgSensorData.hg_ay = highGimu.get_y_gforce();
    highgSensorData.hg_az = highGimu.get_z_gforce();

    #ifdef HIGHGIMU_DEBUG
      Serial.println("------------- HIGH-G THREAD ---------------");
      //high g data
      Serial.print(highgSensorData.hg_ax);
      Serial.print(", ");
      Serial.print(highgSensorData.hg_ay);
      Serial.print(", ");
      Serial.println(highgSensorData.hg_az);
    #endif

    // add the data to the buffer here!
    if (chSemWaitTimeout(&highg_fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        highg_bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&highg_dataMutex);
    highg_fifoArray[highg_fifoHead] = highgSensorData;
    highg_bufferErrors = 0;
    highg_fifoHead = highg_fifoHead < (FIFO_SIZE - 1) ? highg_fifoHead + 1 : 0;
    chSemSignal(&highg_fifoData);


    chMtxUnlock(&highg_dataMutex);

    chThdSleepMilliseconds(6);
  }
}


static THD_FUNCTION(rocket_FSM, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Rocket FSM thread entrance");
    #endif

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO - Acquire lock on data struct!
      chMtxLock(&lowg_dataMutex);
      switch (rocketState) {
            case STATE_INIT:
                // TODO
            break;

            case STATE_IDLE:

                // If high acceleration is observed in z direction...
                //!locking mutex to get data from sensorData struct
                if(lowgSensorData.az > launch_az_thresh) {
                    rocketTimers.launch_time = chVTGetSystemTime();
                    rocketState = STATE_LAUNCH_DETECT;
                }
                //!unlocking &dataMutex mutex

            break;

            case STATE_LAUNCH_DETECT:

                //If the acceleration was too brief, go back to IDLE
                //!locking mutex to get data from sensorData struct
                if (lowgSensorData.az < launch_az_thresh) {
                    rocketState = STATE_IDLE;
                    break;
                }
                //!unlocking &dataMutex mutex

                // measure the length of the burn time (for hysteresis)
                rocketTimers.burn_timer =
                    chVTGetSystemTime() - rocketTimers.launch_time;

                // If the acceleration lasts long enough, boost is detected
                if (rocketTimers.burn_timer > launch_time_thresh) {
                    rocketState = STATE_BOOST;
                    digitalWrite(LED_RED, HIGH);
                }

            break;

            case STATE_BOOST:

            // If low acceleration in the Z direction...
            //!locking mutex to get data from sensorData struct
            if (lowgSensorData.az < coast_thresh) {
                rocketTimers.burnout_time = chVTGetSystemTime();
                rocketState = STATE_BURNOUT_DETECT;
            }
            //!unlocking &dataMutex mutex

            break;

            case STATE_BURNOUT_DETECT:

                //If the low acceleration was too brief, go back to BOOST
                //!locking mutex to get data from sensorData struct
                if (lowgSensorData.az > coast_thresh) {
                    rocketState = STATE_BOOST;
                    break;
                }
                //!unlocking &dataMutex mutex

                // measure the length of the coast time (for hysteresis)
                rocketTimers.coast_timer =
                    chVTGetSystemTime() - rocketTimers.burnout_time;

                // If the low acceleration lasts long enough, coast is detected
                if (rocketTimers.coast_timer > coast_time_thresh) {
                    rocketState = STATE_BOOST;
                }

            break;

            case STATE_COAST:
                // TODO
            break;

            case STATE_APOGEE_DETECT:
                // TODO
            break;

            case STATE_APOGEE:
                // TODO
            break;

            case STATE_DROGUE:
                // TODO
            break;

            case STATE_MAIN:
                // TODO
            break;

        }
        chMtxUnlock(&lowg_dataMutex);

        

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}


void chSetup(){
  //added play_THD for creation

  chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, NULL);
  chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD, NULL);
  chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD, NULL);
  chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO, highgIMU_THD, NULL);
  chThdCreateStatic(lowg_dataLogger_WA, sizeof(lowg_dataLogger_WA), NORMALPRIO, lowg_dataLogger_THD, NULL);
  chThdCreateStatic(highg_dataLogger_WA, sizeof(highg_dataLogger_WA), NORMALPRIO, highg_dataLogger_THD, NULL);
  chThdCreateStatic(gps_dataLogger_WA, sizeof(gps_dataLogger_WA), NORMALPRIO, gps_dataLogger_THD, NULL);
  while(true);
}


void setup() {
  #if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) || defined(HIGHGIMU_DEBUG) || defined(GPS_DEBUG)
    Serial.begin(115200);
    while (!Serial) {}
  #endif

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);

  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_ORANGE, HIGH);

  //TODO: Don't forget this
  Serial.println("------------------------------------------------");

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

    char file_extension[6] = ".csv";

    char lwG_data_name[16] = "lwG_data";
    lowg_dataFile = SD.open(sd_file_namer(lwG_data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    lowg_dataFile.println("ax, ay, az, gx, gy, gz, mx, my, mz, rocketState, timeStamp");

    char highg_data_name[16] = "hgG_data";
    highg_dataFile = SD.open(sd_file_namer(highg_data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    highg_dataFile.println("hg_ax, hg_ay, hg_az, rocketState, timeStamp");

    char gps_data_name[16] = "gps_data";
    gps_dataFile = SD.open(sd_file_namer(gps_data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    gps_dataFile.println("latitude, longitude, altitude, rocketState, GPS Lock, timeStamp");
    
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
