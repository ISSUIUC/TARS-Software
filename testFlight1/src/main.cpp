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


datalogger_THD lowg_datalogger_THD_vars;
datalogger_THD highg_datalogger_THD_vars;
datalogger_THD gps_datalogger_THD_vars;


#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
// #define SERVO_DEBUG

//changed name to account for both high & lowG (logGData)
sensorDataStruct_t gpsSensorData;
sensorDataStruct_t lowgSensorData;
sensorDataStruct_t highgSensorData;

FSM_State rocketState = STATE_INIT;
fsm_struct rocketTimers;


KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();

PWMServo servo_cw; //Servo that controlls roll in the clockwise direction
PWMServo servo_ccw; //Servo that controlls roll in the counter clockwise direction

float flap_drag;
float native_drag;

void round_off_angle(int &value) {
  if (value > 180) {
    value = 180;
  }
  if (value < 0) {
    value = 0;
  }
}


static THD_WORKING_AREA(gps_WA, 512);
static THD_WORKING_AREA(rocket_FSM_WA, 512);
static THD_WORKING_AREA(lowgIMU_WA, 512);
static THD_WORKING_AREA(highgIMU_WA, 512);
static THD_WORKING_AREA(servo_WA, 512);
static THD_WORKING_AREA(lowg_dataLogger_WA, 512);
static THD_WORKING_AREA(highg_dataLogger_WA, 512);
static THD_WORKING_AREA(gps_dataLogger_WA, 512);


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
    if (chSemWaitTimeout(&gps_datalogger_THD_vars.fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        gps_datalogger_THD_vars.bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&gps_datalogger_THD_vars.dataMutex);
    gps_datalogger_THD_vars.fifoArray[gps_datalogger_THD_vars.fifoHead] = gpsSensorData;
    gps_datalogger_THD_vars.bufferErrors = 0;
    gps_datalogger_THD_vars.fifoHead = gps_datalogger_THD_vars.fifoHead < (FIFO_SIZE - 1) ? gps_datalogger_THD_vars.fifoHead + 1 : 0;
    chSemSignal(&gps_datalogger_THD_vars.fifoData);
 

    //!Unlocking &dataMutex
    chMtxUnlock(&gps_datalogger_THD_vars.dataMutex);  

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
    if (chSemWaitTimeout(&highg_datalogger_THD_vars.fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
        highg_datalogger_THD_vars.bufferErrors++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&highg_datalogger_THD_vars.dataMutex);
    highg_datalogger_THD_vars.fifoArray[highg_datalogger_THD_vars.fifoHead] = highgSensorData;
    highg_datalogger_THD_vars.bufferErrors = 0;
    highg_datalogger_THD_vars.fifoHead = highg_datalogger_THD_vars.fifoHead < (FIFO_SIZE - 1) ? highg_datalogger_THD_vars.fifoHead + 1 : 0;
    chSemSignal(&highg_datalogger_THD_vars.fifoData);


    chMtxUnlock(&highg_datalogger_THD_vars.dataMutex);

    chThdSleepMilliseconds(6);
  }
}

//
static THD_FUNCTION(rocket_FSM, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Rocket FSM thread entrance");
    #endif

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO - Acquire lock on data struct!
      chMtxLock(&lowg_datalogger_THD_vars.dataMutex);
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
        chMtxUnlock(&lowg_datalogger_THD_vars.dataMutex);

        

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}

static THD_FUNCTION(servo_THD, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Servo thread entrance");
    #endif
    
    int ccw_angle = 90;
    int cw_angle = 90;
    bool active_control = false;

    chMtxLock(&lowg_datalogger_THD_vars.dataMutex);

    switch(rocketState) {
      case STATE_INIT :
        active_control = true;
        break;
      case STATE_IDLE:
        active_control = true;
        break;
      case STATE_LAUNCH_DETECT :
        active_control = true;
        break;
      case STATE_BOOST :
        active_control = false;
        break;
      case STATE_COAST :
        active_control = true;
        break;
      case STATE_APOGEE_DETECT :
        active_control = false;
        break;
      default :
        active_control = false;
      break;
    }
    // turns active control off if not in takeoff/coast sequence
    if (active_control) {
      cw_angle = lowgSensorData.gz;
      ccw_angle = lowgSensorData.gz;

    } else {
      //Turns active control off if not in coast state.
      cw_angle = 0;
      ccw_angle = 0;
    }
    round_off_angle(cw_angle);
    round_off_angle(ccw_angle);
    servo_cw.write(cw_angle);
    servo_ccw.write(ccw_angle); 
    
    #ifdef SERVO_DEBUG
      Serial.print("\nclockwise: ");
      Serial.print(cw_angle);
      Serial.print(" counterclockwise: ");
      Serial.print(ccw_angle);
    #endif

    chMtxUnlock(&lowg_datalogger_THD_vars.dataMutex);
    chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }

}
void chSetup(){
  //added play_THD for creation

  chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, NULL);
  chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD, NULL);
  chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD, NULL);
  chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO, highgIMU_THD, NULL);
  chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO, servo_THD, NULL);
  chThdCreateStatic(lowg_dataLogger_WA, sizeof(lowg_dataLogger_WA), NORMALPRIO, dataLogger_THD, &lowg_datalogger_THD_vars);
  chThdCreateStatic(highg_dataLogger_WA, sizeof(highg_dataLogger_WA), NORMALPRIO, dataLogger_THD, &highg_datalogger_THD_vars);
  chThdCreateStatic(gps_dataLogger_WA, sizeof(gps_dataLogger_WA), NORMALPRIO, dataLogger_THD, &gps_datalogger_THD_vars);
  while(true);
}


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

  // Setup sensor_type for each sensor
  lowg_datalogger_THD_vars.sensor_type = LOWG_IMU;
  highg_datalogger_THD_vars.sensor_type = HIGHG_IMU;
  gps_datalogger_THD_vars.sensor_type = GPS;

  //SD Card Setup
  if(SD.begin(BUILTIN_SDCARD)){

    char file_extension[6] = ".csv";

    char lwG_data_name[16] = "lwGData";
    lowg_datalogger_THD_vars.dataFile = SD.open(sd_file_namer(lwG_data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    lowg_datalogger_THD_vars.dataFile.println("ax, ay, az, gx, gy, gz, mx, my, mz, timeStamp");
    // Serial.println(lowg_datalogger_THD_vars.dataFile.name());

    char highg_data_name[16] = "hgGData";
    highg_datalogger_THD_vars.dataFile = SD.open(sd_file_namer(highg_data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    highg_datalogger_THD_vars.dataFile.println("hg_ax, hg_ay, hg_az, timeStamp");
    // Serial.println(highg_dataFile.name());

    char gps_data_name[16] = "gpsData";
    gps_datalogger_THD_vars.dataFile = SD.open(sd_file_namer(gps_data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    gps_datalogger_THD_vars.dataFile.println("latitude, longitude, altitude, GPS Lock, timeStamp");
    // Serial.println(gps_dataFile.name());
    
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
