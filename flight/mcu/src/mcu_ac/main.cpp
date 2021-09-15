/* main.cpp
 *   ______  ___     ___    ____
 *  /_  __/ / _ |   / _ \  / __/
 *   / /   / __ |  / , _/ _\ \
 *  /_/   /_/ |_| /_/|_| /___/
 *
 * Active Control Program
 *
 * Illinois Space Society - Avioinics Team
 *
 * Anshuk Chigullapalli
 * Josh Blustein
 * Ayberk Yaraneri
 * David Robbins
 * Matt Taylor
 * Ben Olaivar
 * Colin Kinsey
 * Grace Robbins
 */

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "KX134-1211.h"       //High-G IMU Library
#include "SparkFunLSM9DS1.h"  //Low-G IMU Library
#include "ZOEM8Q0.hpp"        //GPS Library
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
#include "pins.h"
#include "rocketFSM.h"
#include "sensors.h"
#include "servoControl.h"

// datalogger_THD datalogger_THD_vars;

#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

// Create a data struct to hold data from the sensors
sensorDataStruct_t sensorData;

FSM_State rocketState = STATE_INIT;

KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();

PWMServo servo_cw;   // Servo that induces clockwise roll moment
PWMServo servo_ccw;  // Servo that counterclockwisei roll moment

// Create a struct that holds pointers to all the important objects needed by
// the threads
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

/******************************************************************************/
/* ROCKET FINITE STATE MACHINE THREAD                                         */

static THD_FUNCTION(rocket_FSM, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    static rocketFSM stateMachine(pointer_struct);

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Rocket FSM thread entrance");
#endif

        stateMachine.tickFSM();

        chThdSleepMilliseconds(6);  // FSM runs at 100 Hz
    }
}

/******************************************************************************/
/* LOW G IMU THREAD                                                           */

static THD_FUNCTION(lowgIMU_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Low G IMU thread entrance");
#endif

        lowGimuTickFunction(pointer_struct);

        chThdSleepMilliseconds(6);
    }
}

/******************************************************************************/
/* HIGH G IMU THREAD                                                          */

static THD_FUNCTION(highgIMU_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### High G IMU thread entrance");
#endif

        highGimuTickFunction(pointer_struct);

        chThdSleepMilliseconds(6);
    }
}

/******************************************************************************/
/* GPS THREAD                                                                 */

static THD_FUNCTION(gps_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### GPS thread entrance");
#endif

        gpsTickFunction(pointer_struct);

#ifdef THREAD_DEBUG
        Serial.println("### GPS thread exit");
#endif

        chThdSleepMilliseconds(6);  // Sensor DAQ @ ~100 Hz
    }
}

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

static THD_FUNCTION(servo_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;
    bool active_control = false;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Servo thread entrance");
#endif

        servoTickFunction(pointer_struct, &servo_cw, &servo_ccw);

        chThdSleepMilliseconds(6);  // FSM runs at 100 Hz
    }
}

/******************************************************************************/
/* MPU COMMUNICATION THREAD                                                   */

static THD_FUNCTION(mpuComm_THD, arg) {
    // first 3 bytes of packet need to be iss
    (void)arg;

    Serial1.begin(115200);  // Serial interface between MPU and MCU

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### mpuComm thread entrance");
#endif

        //! locking data from sensorData struct
        chMtxLock(&sensor_pointers.dataloggerTHDVarsPointer.dataMutex_lowG);

        digitalWrite(LED_WHITE, HIGH);

        // write transmission code here
        unsigned i = 3;  // because the first 3 indices are already set to be
                         // ISS

        uint8_t *data =
            (uint8_t *)&sensor_pointers.sensorDataPointer->lowG_data;

        //! Unlocking &dataMutex
        chMtxUnlock(&sensor_pointers.dataloggerTHDVarsPointer.dataMutex_lowG);

        mpu_data[0] = 0x49;
        mpu_data[1] = 0x53;
        mpu_data[2] = 0x53;

        for (; i < 3 + sizeof(data); i++) {
            mpu_data[i] = *data;  // de-references to match data types, not sure
                                  // if correct, might send only the first byte
            data++;
        }

        // TODO: Send rocket state too? Is there a mutex for rocket state?

        Serial1.write(mpu_data, sizeof(mpu_data));

        digitalWrite(LED_WHITE, LOW);

        /* for (uint8_t i = 0; i < sizeof(mpu_data); ++i) {
                      Serial.printf("0x%.2X\t", mpu_data[i]);
              }
              Serial.printf("\n\n"); */

        chThdSleepMilliseconds(
            6);  // Set equal sleep time as the other threads, can change
    }
}

/******************************************************************************/
/* DATA LOGGER THREAD                                                   */

static THD_FUNCTION(dataLogger_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("Data Logging thread entrance");
#endif

        dataLoggerTickFunction(pointer_struct);

        chThdSleepMilliseconds(6);
    }
}

/**
 * @brief Starts all of the threads.
 *
 */
void chSetup() {
    // added play_THD for creation

    chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO,
                      rocket_FSM, &sensor_pointers);
    chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD,
                      &sensor_pointers);
    chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD,
                      &sensor_pointers);
    chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO,
                      highgIMU_THD, &sensor_pointers);
    chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO, servo_THD,
                      &sensor_pointers);
    chThdCreateStatic(lowg_dataLogger_WA, sizeof(lowg_dataLogger_WA),
                      NORMALPRIO, dataLogger_THD, &sensor_pointers);
    chThdCreateStatic(mpuComm_WA, sizeof(mpuComm_WA), NORMALPRIO, mpuComm_THD,
                      NULL);

    while (true)
        ;
}

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */
void setup() {
#if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) || \
    defined(HIGHGIMU_DEBUG) || defined(GPS_DEBUG) || defined(SERVO_DEBUG)
    Serial.begin(115200);
    while (!Serial) {
    }
#endif

    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_WHITE, OUTPUT);

    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_ORANGE, HIGH);

    // TODO: Don't forget this
    Serial.println("------------------------------------------------");

    sensor_pointers.lowGimuPointer = &lowGimu;
    sensor_pointers.highGimuPointer = &highGimu;
    sensor_pointers.GPSPointer = &gps;
    sensor_pointers.sensorDataPointer = &sensorData;

    // lowGimu setup
    if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) ==
        false)  // note, we need to sent this our CS pins (defined above)
    {
        digitalWrite(LED_RED, HIGH);
        Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
        while (true)
            ;
    }

    lowGimu.setAccelScale(16);

    // GPS Setup
    gps.beginSPI(ZOEM8Q0_CS);

    // SD Card Setup
    if (SD.begin(BUILTIN_SDCARD)) {
        char file_extension[6] = ".dat";

        char data_name[16] = "data";
        // Initialize SD card
        sensor_pointers.dataloggerTHDVarsPointer.dataFile =
            SD.open(sd_file_namer(data_name, file_extension),
                    O_CREAT | O_WRITE | O_TRUNC);
        // print header to file on sd card that lists each variable that is
        // logged
        sensor_pointers.dataloggerTHDVarsPointer.dataFile.println(
            "ax,ay,az,gx,gy,gz,mx,my,mz,ts_lowg,"
            "hg_ax,hg_ay,hg_az,ts_highg,"
            "latitude,longitude,altitude,GPS Lock,ts_gps,"
            "state_q0,state_q1,state_q2,state_q3,state_x,state_y,state_z,state_"
            "vx,state_vy,state_vz,"
            "state_ax,state_ay,state_az,state_omegax,state_omegay,state_omegaz,"
            "state_latitude,state_longitude,ts_state,"
            "rocketState,ts_RS");
        sensor_pointers.dataloggerTHDVarsPointer.dataFile.flush();
        // Serial.println(lowg_datalogger_THD_vars.dataFile.name());

    } else {
        digitalWrite(LED_RED, HIGH);
        Serial.println("SD Begin Failed. Stalling Program");
        while (true)
            ;
    }

    // Servo Setup
    servo_cw.attach(BALL_VALVE_1_PIN, 770,
                    2250);  // TODO: MAKE SURE TO CHANGE PINS
    servo_ccw.attach(BALL_VALVE_2_PIN, 770, 2250);

    Serial.println("Starting ChibiOS");
    chBegin(chSetup);
    while (true)
        ;
}

void loop() {
    // not used
}
