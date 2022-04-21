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
#include <RH_RF95.h>
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
#include "telemetry.h"
#include <RHSoftwareSPI.h>

#define RFM95_CS 41
#define RFM95_RST 20
#define RFM95_INT 40


//Make sure to change these pinout depending on wiring
//Don't forget to change the ini file to build the correct main file
// #define RFM95_CS 10
// #define RFM95_RST 15
// #define RFM95_INT 17
// #define RFM95_EN 14

// // Change to 434.0 or other frequency, must match RX's freq!
// #define RF95_FREQ 434.0

// datalogger_THD datalogger_THD_vars;

//#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

// Create a data struct to hold data from the sensors
sensorDataStruct_t sensorData;


FSM_State rocketState = STATE_INIT;

RHSoftwareSPI s;


RH_RF95 rf95(RFM95_CS, RFM95_INT, s);

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
    // int packetnum = 0;
    Telemetry tlm;
    while(true) {
        tlm.transmit(rf95);
        chThdSleepMilliseconds(1);
//         Serial.println("Sending to rf95_server");
//   // Send a message to rf95_server
  
//         char radiopacket[20] = "Hey bestie #      ";
//         itoa(packetnum++, radiopacket+13, 10);
//         Serial.print("Sending "); Serial.println(radiopacket);
//         radiopacket[19] = 0;
        
//         Serial.println("Sending..."); delay(10);
//         rf95.send((uint8_t *)radiopacket, 20);

//         Serial.println("Waiting for packet to complete..."); delay(10);
//         rf95.waitPacketSent();
//         // Now wait for a reply
//         uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//         uint8_t len = sizeof(buf);
//                                                 //test without delay
//         Serial.println("Waiting for reply..."); //delay(10);
//         if (rf95.waitAvailableTimeout(1000))
//         { 
//             // Should be a reply message for us now   
//             if (rf95.recv(buf, &len))
//         {
//             Serial.print("Got reply: ");
//             Serial.println((char*)buf);
//             Serial.print("RSSI: ");
//             Serial.println(rf95.lastRssi(), DEC);    
//             }
//             else
//             {
//             Serial.println("Receive failed");
//             }
//         }
//         else
//         {
//             Serial.println("No reply, is there a listener around?");
//         }
//         chThdSleepMilliseconds(1000);
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
        // Serial.println("abcd1");
        // Serial.println("abcd2");
        // Serial.println("abcd3");
        // Serial.println("abcd4");
        // Serial.println("abcd5");
        // stateMachine.tickFSM();
        // delay(100);

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
    // chThdCreateStatic(telemetry_WA, sizeof(telemetry_WA), NORMALPRIO + 1,
    //                   telemetry_THD, &sensor_pointers);
    chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO + 1,
                      rocket_FSM, &sensor_pointers);
    // // chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO + 1, gps_THD,
    // //                   &sensor_pointers);
    chThdCreateStatic(barometer_WA, sizeof(barometer_WA), NORMALPRIO + 1,
                      barometer_THD, &sensor_pointers);
    chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO + 1,
                      lowgIMU_THD, &sensor_pointers);
    chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO + 1,
                      highgIMU_THD, &sensor_pointers);
    chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO + 1, servo_THD,
                      &sensor_pointers);
    // chThdCreateStatic(dataLogger_WA, sizeof(dataLogger_WA), NORMALPRIO + 1,
    //                   dataLogger_THD, &sensor_pointers);
    // chThdCreateStatic(mpuComm_WA, sizeof(mpuComm_WA), NORMALPRIO + 1,
    //                   mpuComm_THD, NULL);

    while (true)
        ;
}

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */

void setup() {
    // Telemetry tlm;
    int32_t temperature;
    

// #if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) ||     \
//     defined(BAROMETER_DEBUG) || defined(HIGHGIMU_DEBUG) || \
//     defined(GPS_DEBUG) || defined(SERVO_DEBUG)
// #endif
    // pinMode(LED_BLUE, OUTPUT);
    // pinMode(LED_RED, OUTPUT);
    // pinMode(LED_ORANGE, OUTPUT);
    // pinMode(LED_WHITE, OUTPUT);

    // digitalWrite(LED_BLUE, HIGH);
    // digitalWrite(LED_ORANGE, HIGH);
    Serial.begin(9600);
    while(!Serial);
    pinMode(LSM9DS1_AG_CS, OUTPUT);
    digitalWrite(LSM9DS1_AG_CS, HIGH);
    pinMode(LSM9DS1_M_CS, OUTPUT);
    digitalWrite(LSM9DS1_M_CS, HIGH);
    pinMode(ZOEM8Q0_CS, OUTPUT);
    digitalWrite(ZOEM8Q0_CS, HIGH);
    pinMode(MS5611_CS, OUTPUT);
    digitalWrite(MS5611_CS, HIGH);
    pinMode(KX134_CS, OUTPUT);
    digitalWrite(KX134_CS, HIGH);
    pinMode(H3LIS331DL_CS, OUTPUT);
    digitalWrite(H3LIS331DL_CS, HIGH);
    pinMode(RFM95_CS, OUTPUT);
    digitalWrite(RFM95_CS, HIGH);

    // TODO: Don't forget this
    //Serial.println("------------------------------------------------");

    sensor_pointers.lowGimuPointer = &lowGimu;
    sensor_pointers.highGimuPointer = &highGimu;
    sensor_pointers.barometerPointer = &barometer;
    sensor_pointers.GPSPointer = &gps;
    sensor_pointers.sensorDataPointer = &sensorData;
    sensor_pointers.abort = false;

    SPI.begin();
    SPI1.setMISO(39);

    // // Initialize barometer
    // barometer.init();

    if(!highGimu.beginSPICore(KX134_CS, 1000000, SPI)){
        // digitalWrite(LED_RED, HIGH);
        Serial.println("Failed to communicate with KX134. Stalling Program");
        while (true)
            ;
    }


    pinMode(RFM95_RST, OUTPUT);
    pinMode(RFM95_EN, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    digitalWrite(RFM95_EN, HIGH);

    // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);


    Serial.println("Starting ChibiOS");
    chBegin(chSetup);
    while (true)
        ;
}

void loop() {
    // not used
}