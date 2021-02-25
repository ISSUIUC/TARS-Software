#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

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
static MUTEX_DECL(dataMutex);

//#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
#define SERVO_DEBUG

//changed name to account for both high & lowG (logGData)
dataStruct_t sensorData;

FSM_State rocketState = STATE_INIT;
fsm_struct rocketTimers;

File dataFile;

KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();

Servo servo_cw; //Servo that controlls roll in the clockwise direction
Servo servo_ccw; //Servo that controlls roll in the counter clockwise direction

int servo_cw_angle; //The current angle of the clockwise roll controlled servo.
int servo_ccw_angle; //The current angle of the counter clockwise roll controlled servo.
float flap_drag;
float native_drag;


static THD_WORKING_AREA(gps_WA, 512);
static THD_WORKING_AREA(rocket_FSM_WA, 512);
static THD_WORKING_AREA(lowgIMU_WA, 512);
static THD_WORKING_AREA(highgIMU_WA, 512);
static THD_WORKING_AREA(servo_WA, 512);

static THD_FUNCTION(gps_THD, arg){
  (void)arg;
  while(true){
    
    #ifdef THREAD_DEBUG
      Serial.println("### GPS thread entrance");
    #endif

    //!locking data from sensorData struct
    chMtxLock(&dataMutex);

    sensorData.timeStamp = chVTGetSystemTime();

    chSysLock();
    gps.update_data();
    chSysUnlock();

    //Have the availability to wait until a lock is aquired with gps.get_position_lock();
    sensorData.latitude = gps.get_latitude();
    sensorData.longitude = gps.get_longitude();
    sensorData.altitude = gps.get_altitude();
    sensorData.posLock = gps.get_position_lock();
    

    if(sensorData.posLock == true){
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
        Serial.println(sensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(sensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(sensorData.altitude);
      } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(sensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(sensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(sensorData.altitude);
	      Serial.println("");
      }
    #endif

    logData(&dataFile, &sensorData, rocketState);

    //!Unlocking &dataMutex
    chMtxUnlock(&dataMutex);  

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}


static THD_FUNCTION(lowgIMU_THD, arg) {
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Low G IMU thread entrance");
    #endif

    chMtxLock(&dataMutex);

    chSysLock();
    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();
    chSysUnlock();

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

    #ifdef LOWGIMU_DEBUG
      Serial.println("------------- LOW-G THREAD ---------------");
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
    #endif

    chMtxUnlock(&dataMutex);

    chThdSleepMilliseconds(6);
  }
}


static THD_FUNCTION(highgIMU_THD, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### High G IMU thread entrance");
    #endif

    chMtxLock(&dataMutex);

    chSysLock();
    highGimu.update_data();
    chSysUnlock();

    //addition for highG IMU
    sensorData.hg_ax = highGimu.get_x_gforce();
    sensorData.hg_ay = highGimu.get_y_gforce();
    sensorData.hg_az = highGimu.get_z_gforce();

    #ifdef HIGHGIMU_DEBUG
      Serial.println("------------- HIGH-G THREAD ---------------");
      //high g data
      Serial.print(sensorData.hg_ax);
      Serial.print(", ");
      Serial.print(sensorData.hg_ay);
      Serial.print(", ");
      Serial.println(sensorData.hg_az);
    #endif

    chMtxUnlock(&dataMutex);

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
      chMtxLock(&dataMutex);
      switch (rocketState) {
            case STATE_INIT:
                // TODO
            break;

            case STATE_IDLE:

                // If high acceleration is observed in z direction...
                //!locking mutex to get data from sensorData struct
                if(sensorData.az > launch_az_thresh) {
                    rocketTimers.launch_time = chVTGetSystemTime();
                    rocketState = STATE_LAUNCH_DETECT;
                }
                //!unlocking &dataMutex mutex

            break;

            case STATE_LAUNCH_DETECT:

                //If the acceleration was too brief, go back to IDLE
                //!locking mutex to get data from sensorData struct
                if (sensorData.az < launch_az_thresh) {
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
            if (sensorData.az < coast_thresh) {
                rocketTimers.burnout_time = chVTGetSystemTime();
                rocketState = STATE_BURNOUT_DETECT;
            }
            //!unlocking &dataMutex mutex

            break;

            case STATE_BURNOUT_DETECT:

                //If the low acceleration was too brief, go back to BOOST
                //!locking mutex to get data from sensorData struct
                if (sensorData.az > coast_thresh) {
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
        chMtxUnlock(&dataMutex);

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}

static THD_FUNCTION(servo_THD, arg){
  (void)arg;
  while(true){

    #ifdef SERVO_DEBUG
      Serial.println("### Servo thread entrance");
    #endif
    int ccw_angle = 90;
    int cw_angle = 90;

    chMtxLock(&dataMutex);

    switch(rocketState) {
      case STATE_INIT :
        //todo
        break;
      case STATE_IDLE:
        //todo
        break;
      case STATE_LAUNCH_DETECT :
        //todo
        break;
      case STATE_BOOST :
        //todo
        break;
      case STATE_BURNOUT_DETECT :
        //todo
        break;
      case STATE_COAST :
        //todo
        break;
      case STATE_APOGEE_DETECT :
        //todo
        break;
      default :
      break;
    }

    servo_cw.write(cw_angle);
    servo_ccw.write(ccw_angle);  
    servo_cw_angle = cw_angle;
    servo_ccw_angle = ccw_angle;

    chMtxUnlock(&dataMutex);
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
    init_dataLog(&dataFile);
  }
  else {
    digitalWrite(LED_RED, HIGH);
    Serial.println("SD Begin Failed. Stalling Program");
    while(true);
  }

  //Servo Setup
  servo_cw.attach(1);
  servo_ccw.attach(2);

  Serial.println("Starting ChibiOS");
  chBegin(chSetup);
  while(true);

  
}

void loop() {
  // not used
}
