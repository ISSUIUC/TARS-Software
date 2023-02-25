/** @file main.cpp
 *   ______  ___     ___    ____
 *  /_  __/ / _ |   / _ \  / __/
 *   / /   / __ |  / , _/ _\ \
 *  /_/   /_/ |_| /_/|_| /___/
 *
 * Rocket Flight Code
 *
 * Illinois Space Society - Software + Active Controls + Telemetry
 *
 * Anshuk Chigullapalli
 * Josh Blustein
 * Ayberk Yaraneri
 * David Robbins
 * Matt Taylor
 * Ben Olaivar
 * Colin Kinsey
 * Grace Robbins
 * Nicholas Phillips
 * Gautam Dayal
 * Patrick Marschoun
 * Parth Shrotri
 * Jeffery Zhou
 * Karnap Patel
 * Magilan Sendhil
 */

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SPI.h>
// #include <LSM6.h>
#include <SparkFunLSM6DS3.h>

#include "mcu_main/Abort.h"
#include "mcu_main/SDLogger.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/finite-state-machines/rocketFSM.h"
#include "mcu_main/gnc/ActiveControl.h"
#include "mcu_main/gnc/kalmanFilter.h"
#include "mcu_main/pins.h"
#include "mcu_main/sensors/sensors.h"
#include "mcu_main/telemetry.h"

#define THREAD_DEBUG
//#define SERVO_DEBUG

/******************************************************************************/
/* TELEMETRY THREAD                                         */

static THD_FUNCTION(telemetry_buffering_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### telemetry buffering thread entrance");
#endif
        tlm.bufferData();
        chThdSleepMilliseconds(80);
    }
}

/******************************************************************************/
/* TELEMETRY THREAD                                         */

static THD_FUNCTION(telemetry_sending_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### telemetry sending thread entrance");
#endif
        tlm.transmit();

        if (tlm.abort) {
            startAbort();
        }
        chThdSleepMilliseconds(200);
        // transmit has a sleep in it
    }
}

/******************************************************************************/
/* ROCKET FINITE STATE MACHINE THREAD                                         */

static THD_FUNCTION(rocket_FSM_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Rocket FSM thread entrance");
#endif

        fsmCollection.tick();

        rocketStateData<4> fsm_state = fsmCollection.getStates();
        dataLogger.pushRocketStateFifo(fsm_state);

        chThdSleepMilliseconds(6);  // FSM runs at 100 Hz
    }
}

/******************************************************************************/
/* LOW G IMU THREAD                                                           */

static THD_FUNCTION(lowgIMU_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Low G IMU thread entrance");
#endif

        lowG.update();

        chThdSleepMilliseconds(6);
    }
}

/******************************************************************************/
/* BAROMETER THREAD                                                           */

static THD_FUNCTION(barometer_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Barometer thread entrance");
#endif

        barometer.refresh();

        chThdSleepMilliseconds(6);
    }
}

/******************************************************************************/
/* HIGH G IMU THREAD                                                          */

static THD_FUNCTION(highgIMU_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### High G IMU thread entrance");
#endif
        highG.update();

        chThdSleepMilliseconds(20);  // highg reads at 50 hz
    }
}

/******************************************************************************/
/* GPS THREAD                                                                 */

static THD_FUNCTION(gps_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### GPS thread entrance");
#endif
        gps.update();

#ifdef THREAD_DEBUG
        Serial.println("### GPS thread exit");
#endif

        chThdSleepMilliseconds(190);  // Read the gps @ ~5 Hz
    }
}

/******************************************************************************/
/* KALMAN FILTER THREAD                                                       */

static THD_FUNCTION(kalman_THD, arg) {
    kalmanFilter.Initialize();

    systime_t last = chVTGetSystemTime();

    while (true) {
        kalmanFilter.kfTickFunction(TIME_I2MS(chVTGetSystemTime() - last), 13.0);
        last = chVTGetSystemTime();

        chThdSleepMilliseconds(50);
    }
}

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

static THD_FUNCTION(servo_THD, arg) {
    activeController.init();

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Servo thread entrance");
#endif
        activeController.ctrlTickFunction();

        chThdSleepMilliseconds(6);  // FSM runs at 166 Hz
    }
}

/******************************************************************************/
/* MPU COMMUNICATION THREAD                                                   */

static THD_FUNCTION(voltage_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### voltage thread entrance");
#endif
        voltage.read();

        chThdSleepMilliseconds(6);  // Set equal sleep time as the other threads, can change
    }
}

/******************************************************************************/
/* DATA LOGGER THREAD                                                   */

static THD_FUNCTION(dataLogger_THD, arg) {
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("Data Logging thread entrance");
#endif

        sd_logger.update();

        chThdSleepMilliseconds(6);
    }
}

static THD_WORKING_AREA(barometer_WA, 8192);
static THD_WORKING_AREA(gps_WA, 8192);
static THD_WORKING_AREA(rocket_FSM_WA, 8192);
static THD_WORKING_AREA(lowgIMU_WA, 8192);
static THD_WORKING_AREA(highgIMU_WA, 8192);
static THD_WORKING_AREA(servo_WA, 8192);
static THD_WORKING_AREA(dataLogger_WA, 8192);
static THD_WORKING_AREA(voltage_WA, 8192);
static THD_WORKING_AREA(telemetry_sending_WA, 8192);
static THD_WORKING_AREA(telemetry_buffering_WA, 8192);
static THD_WORKING_AREA(kalman_WA, 8192);

#define START_THREAD(NAME) chThdCreateStatic(NAME##_WA, sizeof(NAME##_WA), NORMALPRIO + 1, NAME##_THD, nullptr)
/**
 * @brief Starts all of the threads.
 */
void chSetup() {
    START_THREAD(telemetry_sending);
    START_THREAD(telemetry_buffering);
    START_THREAD(rocket_FSM);
    START_THREAD(gps);
    START_THREAD(lowgIMU);
    START_THREAD(barometer);
    START_THREAD(highgIMU);
    START_THREAD(servo);
    START_THREAD(dataLogger);
    START_THREAD(voltage);
    START_THREAD(kalman);

    while (true)
        ;
}

#undef START_THREAD

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */

void setup() {


    Serial.begin(9600);
    // Stall until serial monitor opened
    while (!Serial);
    Serial.println("Starting SPI...");

    SPI.begin();
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);

    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    pinMode(MS5611_CS, OUTPUT);
    digitalWrite(MS5611_CS, HIGH);

    pinMode(KX134_CS, OUTPUT);
    digitalWrite(KX134_CS, HIGH);
  

    pinMode(RFM96_CS, OUTPUT);
    digitalWrite(RFM96_CS, HIGH);

    pinMode(LSM6DSLTR, OUTPUT);
    digitalWrite(LSM6DSLTR, HIGH);

    pinMode(LIS3MDL_CS, OUTPUT);
    digitalWrite(LIS3MDL_CS, HIGH);

    // MS5611 barometer{MS5611_CS};
    // barometer.init();
    
    // while (1) {
    //     barometer.read(12);
    //     float temp = barometer.getTemperature() *
    //     0.01;
    //     float pressure = barometer.getPressure()*0.01;
    //     Serial.println(pressure);
    // }

    QwiicKX134 highG;
    // highG.beginSPICore(KX134_CS, 1000000, SPI);
    highG.beginSPI(KX134_CS);
    highG.initialize(DEFAULT_SETTINGS);
    highG.setRange(3);
    while (1) {
        auto data = highG.getAccelData();
        Serial.println(data.zData);
    }

    LSM6 lowG;
=  

    







    Serial.println("chibios begin");
    chBegin(chSetup);
    while (true)
        ;
}

void loop() {
    // not used
}
