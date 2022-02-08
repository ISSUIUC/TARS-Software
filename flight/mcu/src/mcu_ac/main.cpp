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

#include "ActiveControl.h"
#include "KX134-1211.h"  //High-G IMU Library
#include "MS5611.h"      //Barometer library
#include "ServoControl.h"
#include "SparkFunLSM9DS1.h"                       //Low-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"  //GPS Library
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
#include "pins.h"
#include "rocketFSM.h"
#include "sensors.h"

// datalogger_THD datalogger_THD_vars;

//#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

// Create a data struct to hold data from the sensors
sensorDataStruct_t sensorData;

FSM_State rocketState = STATE_INIT;

KX134 highGimu;
LSM9DS1 lowGimu;
SFE_UBLOX_GNSS gps;

MS5611 barometer{MS5611_CS};

PWMServo servo_cw;   // Servo that induces clockwise roll moment
PWMServo servo_ccw;  // Servo that counterclockwisei roll moment

// Create a struct that holds pointers to all the important objects needed by
// the threads
pointers sensor_pointers;

uint8_t mpu_data[71];

static THD_WORKING_AREA(barometer_WA, 8192);
static THD_WORKING_AREA(gps_WA, 8192);
static THD_WORKING_AREA(rocket_FSM_WA, 8192);
static THD_WORKING_AREA(lowgIMU_WA, 8192);
static THD_WORKING_AREA(highgIMU_WA, 8192);
static THD_WORKING_AREA(servo_WA, 8192);
static THD_WORKING_AREA(dataLogger_WA, 8192);
static THD_WORKING_AREA(mpuComm_WA, 8192);
static THD_WORKING_AREA(telemetry_WA, 8192);

/******************************************************************************/
/* TELEMETRY THREAD                                         */

static THD_FUNCTION(telemetry_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    while(true) {
        Serial.println("thread");
        chThdSleepMilliseconds(100);
    }
}

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
/* BAROMETER THREAD                                                           */

static THD_FUNCTION(barometer_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Barometer thread entrance");
#endif

        barometerTickFunction(pointer_struct);

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

        chThdSleepMilliseconds(80);  // Read the gps @ ~10 Hz
    }
}

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

static THD_FUNCTION(servo_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    ActiveControl ac(pointer_struct, &servo_ccw, &servo_cw);

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Servo thread entrance");
#endif

        ac.acTickFunction();

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
    chThdCreateStatic(telemetry_WA, sizeof(telemetry_WA), NORMALPRIO + 1,
                      telemetry_THD, &sensor_pointers);

    while (true)
        ;
}

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */

void setup() {
    int32_t temperature;

#if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) ||     \
    defined(BAROMETER_DEBUG) || defined(HIGHGIMU_DEBUG) || \
    defined(GPS_DEBUG) || defined(SERVO_DEBUG)
    Serial.begin(115200);
    while (!Serial) {
    }
#endif
    while(!Serial);
    Serial.println("Starting ChibiOS");
    chBegin(chSetup);
    while (true)
        ;
}

void loop() {
    // not used
}