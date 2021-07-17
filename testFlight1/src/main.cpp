#ifndef MAIN_CPP
#define MAIN_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"

#include "sensors.cpp"
#include "servo.cpp"


// datalogger_THD datalogger_THD_vars;


#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

// Create a data struct to hold data from the sensors
sensorDataStruct_t sensorData;


FSM_State rocketState = STATE_INIT;
fsm_struct rocketTimers;


KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();


// Create a struct that holds pointers to all the important objects needed by the threads
pointers sensor_pointers;


uint8_t mpu_data[71];

static THD_WORKING_AREA(gps_WA, 512);
static THD_WORKING_AREA(rocket_FSM_WA, 512);
static THD_WORKING_AREA(lowgIMU_WA, 512);
static THD_WORKING_AREA(highgIMU_WA, 512);
static THD_WORKING_AREA(servo_WA, 512);
static THD_WORKING_AREA(lowg_dataLogger_WA, 512);
static THD_WORKING_AREA(highg_dataLogger_WA, 512);
static THD_WORKING_AREA(gps_dataLogger_WA, 512);
static THD_WORKING_AREA(mpuComm_WA, 512);

static THD_FUNCTION(mpuComm_THD, arg){
  //first 3 bytes of packet need to be iss
  (void)arg;

  Serial1.begin(115200); // Serial interface between MPU and MCU

  while (true) {

    #ifdef THREAD_DEBUG
      Serial.println("### mpuComm thread entrance");
    #endif

    //!locking data from sensorData struct
    chMtxLock(&sensor_pointers.dataloggerTHDVarsPointer.dataMutex_lowG);

    digitalWrite(LED_WHITE, HIGH);

    //write transmission code here
    unsigned i = 3; //because the first 3 indices are already set to be ISS 

    uint8_t* data = (uint8_t*) &sensor_pointers.sensorDataPointer->lowG_data;

    //!Unlocking &dataMutex
    chMtxUnlock(&sensor_pointers.dataloggerTHDVarsPointer.dataMutex_lowG);

    mpu_data[0] = 0x49;
    mpu_data[1] = 0x53;
    mpu_data[2] = 0x53;

    for (; i < 3 + sizeof(data); i++) {
      mpu_data[i] = *data; //de-references to match data types, not sure if correct, might send only the first byte
      data++;
    }

    //TODO: Send rocket state too? Is there a mutex for rocket state?

    Serial1.write(mpu_data, sizeof(mpu_data));

    digitalWrite(LED_WHITE, LOW);

    /* for (uint8_t i = 0; i < sizeof(mpu_data); ++i) {
		  Serial.printf("0x%.2X\t", mpu_data[i]);
	  }
	  Serial.printf("\n\n"); */
  
    

    chThdSleepMilliseconds(6); //Set equal sleep time as the other threads, can change  
  }
}





//
static THD_FUNCTION(rocket_FSM, arg){
  struct pointers *pointer_struct = (struct pointers *)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Rocket FSM thread entrance");
    #endif

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // Lock mutexes for data used in switch
      chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
      chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
      chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS); //TODO, used for state init?
      switch (pointer_struct->sensorDataPointer->rocketState_data.rocketState) {
            case STATE_INIT:
                if(pointer_struct->GPSPointer->get_position_lock()) { //if GPS lock detected, go to state Idle
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_IDLE;
                }
            break;
 
            case STATE_IDLE:
 
                // If high acceleration is observed in z direction...
                if(pointer_struct->sensorDataPointer->state_data.state_az > launch_az_thresh) {
                    rocketTimers.launch_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LAUNCH_DETECT;
                }
 
            break;
 
            case STATE_LAUNCH_DETECT:
 
                //If the acceleration was too brief, go back to IDLE
                if (pointer_struct->sensorDataPointer->state_data.state_az < launch_az_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_IDLE;
                    break;
                }
 
                // measure the length of the burn time (for hysteresis)
                rocketTimers.burn_timer =
                    chVTGetSystemTime() - rocketTimers.launch_time;
 
                // If the acceleration lasts long enough, boost is detected
                if (rocketTimers.burn_timer > launch_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_BOOST;
                    digitalWrite(LED_RED, HIGH);
                }
 
            break;
 
            case STATE_BOOST:
 
            // If low acceleration in the Z direction...
            if (pointer_struct->sensorDataPointer->state_data.state_az < coast_thresh) {
                rocketTimers.burnout_time = chVTGetSystemTime();
                pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_BURNOUT_DETECT;
            }
 
            break;
 
            case STATE_BURNOUT_DETECT:
 
                //If the low acceleration was too brief, go back to BOOST
                if (pointer_struct->sensorDataPointer->state_data.state_az > coast_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_BOOST;
                    break;
                }
 
                // measure the length of the coast time (for hysteresis)
                rocketTimers.coast_timer =
                    chVTGetSystemTime() - rocketTimers.burnout_time;
 
                // If the low acceleration lasts long enough, coast is detected
                if (rocketTimers.coast_timer > coast_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_COAST;
                }
 
            break;
 
            case STATE_COAST:
                //if velocity < 0
                if (pointer_struct->sensorDataPointer->state_data.state_vz < apogee_thresh) {
                    rocketTimers.apogee_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE_DETECT;
                }
 
            break;
 
            case STATE_APOGEE_DETECT:
                //If the negative velocity was too brief, go back to coast
                if (pointer_struct->sensorDataPointer->state_data.state_vz > apogee_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_COAST;
                    break;
                }
 
                // measure the length of the apogee time
                rocketTimers.apogee_timer =
                    chVTGetSystemTime() - rocketTimers.apogee_time;
 
                // If the < 0 velocity lasts long enough, apogee is detected
                if (rocketTimers.coast_timer > apogee_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE;
                }
            break;
 
            case STATE_APOGEE:
                //If the rocket has higher acceleration
                if (pointer_struct->sensorDataPointer->state_data.state_vz > drogue_thresh) {
                    rocketTimers.drogue_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_DROGUE_DETECT;
                }
            break;
 
            case STATE_DROGUE_DETECT:
                //If the acceleration was too brief, go back to apogee
                if (pointer_struct->sensorDataPointer->state_data.state_vz < drogue_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE;
                    break;
                }
 
                // measure the length of drogue time
                rocketTimers.apogee_timer =
                    chVTGetSystemTime() - rocketTimers.drogue_time;
 
                // If the acceleration lasts long enough, drogue is detected
                if (rocketTimers.drogue_timer > drogue_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_DROGUE;
                }
            break;
 
            case STATE_DROGUE:
                //If the rocket has higher acceleration
                //!locking mutex to get data from sensorData struct
                if (pointer_struct->sensorDataPointer->state_data.state_vz > main_thresh) {
                    rocketTimers.main_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_MAIN_DETECT;
                }
            break;
 
            case STATE_MAIN_DETECT:
                //If the acceleration was too brief, go back to drogue
                if (pointer_struct->sensorDataPointer->state_data.state_vz < drogue_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_DROGUE;
                    break;
                }
 
                // measure the length of the main time
                rocketTimers.main_timer =
                    chVTGetSystemTime() - rocketTimers.main_time;
 
                // If the low velocity lasts long enough, main is detected
                if (rocketTimers.main_timer > main_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_MAIN;
                }
            break;
 
            case STATE_MAIN:
                //If the rocket has 0 velocity
                if (pointer_struct->sensorDataPointer->state_data.state_vz > landed_thresh) {
                    rocketTimers.landing_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LANDED_DETECT;
                }
            break;
 
            case STATE_LANDED_DETECT:
                //If the 0 velocity was too brief, go back to main
                if (pointer_struct->sensorDataPointer->state_data.state_vz < landed_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_MAIN;
                    break;
                }
 
                // measure the length of the landing
                rocketTimers.landing_timer =
                    chVTGetSystemTime() - rocketTimers.landing_time;
 
                // If the 0 velocity lasts long enough, landed is detected
                if (rocketTimers.landing_timer > landing_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LANDED;
                }
            break;
 
            case STATE_LANDED:
                // TODO
            break;

        }
        // Update timestamp for when rocket state was polled
        pointer_struct->sensorDataPointer->rocketState_data.timeStamp_RS = chVTGetSystemTime();

        // Unlock mutexes used during the switch statement
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS); //?
        

        // check that data can be writen to the rocket state buffer
        if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer.fifoSpace_RS, TIME_IMMEDIATE) != MSG_OK) {
            pointer_struct->dataloggerTHDVarsPointer.bufferErrors_RS++;
            digitalWrite(LED_BUILTIN, HIGH);
            continue;
        }
        // Write rocket state data to the buffer
        chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
        pointer_struct->dataloggerTHDVarsPointer.fifoArray[pointer_struct->dataloggerTHDVarsPointer.fifoHead_GPS].rocketState_data = pointer_struct->sensorDataPointer->rocketState_data;
        pointer_struct->dataloggerTHDVarsPointer.bufferErrors_RS = 0;
        pointer_struct->dataloggerTHDVarsPointer.fifoHead_RS = pointer_struct->dataloggerTHDVarsPointer.fifoHead_RS < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer.fifoHead_RS + 1 : 0;
        chSemSignal(&pointer_struct->dataloggerTHDVarsPointer.fifoData_RS);
        //!Unlocking &dataMutex for rocket state
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);

        

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}

/**
 * @brief Starts all of the threads.
 * 
 */
void chSetup(){
  //added play_THD for creation

  chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, &sensor_pointers);
  chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD, &sensor_pointers);
  chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD, &sensor_pointers);
  chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO, highgIMU_THD, &sensor_pointers);
  chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO, servo_THD, &sensor_pointers);
  chThdCreateStatic(lowg_dataLogger_WA, sizeof(lowg_dataLogger_WA), NORMALPRIO, dataLogger_THD, &sensor_pointers);
  chThdCreateStatic(mpuComm_WA, sizeof(mpuComm_WA), NORMALPRIO, mpuComm_THD, NULL);

  while(true);
}

/**
 * @brief Handles all configuration necessary before the threads start.
 * 
 */
void setup() {
  #if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) || defined(HIGHGIMU_DEBUG) || defined(GPS_DEBUG) || defined(SERVO_DEBUG)
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


  sensor_pointers.lowGimuPointer = &lowGimu;
  sensor_pointers.highGimuPointer = &highGimu;
  sensor_pointers.GPSPointer = &gps;
  sensor_pointers.sensorDataPointer = &sensorData;

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

    char file_extension[6] = ".dat";

    char data_name[16] = "data";
    // Initialize SD card
    sensor_pointers.dataloggerTHDVarsPointer.dataFile = SD.open(sd_file_namer(data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    // print header to file on sd card that lists each variable that is logged
    sensor_pointers.dataloggerTHDVarsPointer.dataFile.println("ax,ay,az,gx,gy,gz,mx,my,mz,ts_lowg,"
                                                               "hg_ax,hg_ay,hg_az,ts_highg,"
                                                               "latitude,longitude,altitude,GPS Lock,ts_gps,"
                                                               "state_q0,state_q1,state_q2,state_q3,state_x,state_y,state_z,state_vx,state_vy,state_vz,"
                                                               "state_ax,state_ay,state_az,state_omegax,state_omegay,state_omegaz,state_latitude,state_longitude,ts_state,"
                                                               "rocketState,ts_RS");
    sensor_pointers.dataloggerTHDVarsPointer.dataFile.flush();
    // Serial.println(lowg_datalogger_THD_vars.dataFile.name());

    
  }
  else {
    digitalWrite(LED_RED, HIGH);
    Serial.println("SD Begin Failed. Stalling Program");
    while(true);
  }

  //Servo Setup
  servo_cw.attach(BALL_VALVE_1_PIN, 770, 2250); //TODO: MAKE SURE TO CHANGE PINS
  servo_ccw.attach(BALL_VALVE_2_PIN, 770, 2250);

  Serial.println("Starting ChibiOS");
  chBegin(chSetup);
  while(true);
 
}

void loop() {
  // not used
}

#endif
