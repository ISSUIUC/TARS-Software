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

#include "mcu_main/Rt.h"
#ifndef ENABLE_SILSIM_MODE
#include <Arduino.h>
#include <PWMServo.h>
#include <SPI.h>
#include <pb_decode.h>
#include <Wire.h>
#endif

#include "mcu_main/Abort.h"
#include "mcu_main/SDLogger.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/finite-state-machines/rocketFSM.h"
#include "mcu_main/gnc/ActiveControl.h"
#include "mcu_main/gnc/kalmanFilter.h"
#include "mcu_main/pins.h"
#include "mcu_main/sensors/sensors.h"
#include "mcu_main/telemetry.h"
#include "mcu_main/error.h"
#include "mcu_main/buzzer/buzzer.h"
#include "mcu_main/debug.h"
#include "mcu_main/hilsim/hilsimpacket.pb.h"

HILSIMPacket hilsim_reader;

#if defined(ENABLE_HIGH_G) || defined(ENABLE_ORIENTATION) || defined(ENABLE_BAROMETER) || defined(ENABLE_LOW_G) || \
    defined(ENABLE_MAGNETOMETER) || defined(ENABLE_GAS)
#define ENABLE_SENSOR_FAST
#endif

#ifdef ENABLE_HILSIM_MODE
static THD_FUNCTION(hilsim_THD, arg) {
    // Size of protobuf packet size
    // Check src/mcu_main/hilsim/README.md for more info
    const unsigned int packet_size = 70;
    // Creating array for data to be read into
    char data_read[packet_size];
    int fields_to_read = 19;
    Serial.setTimeout(30);
    Serial.println("[TARS] Hardware-in-Loop Test Commenced");

    while (1) {
        int bytes_read = Serial.readBytes(data_read, packet_size);
        if (bytes_read != 0 && bytes_read != packet_size) {
            Serial.print(bytes_read);
            Serial.println(" bytes read, not reading sufficient bytes");
            int dummy = packet_size - bytes_read;
            // Then read mod seventy until the next bit
            int s = Serial.readBytes(data_read, dummy);
            Serial.print(s);
            Serial.println(" trashed bytes");
        } else if (bytes_read > 0) {
            Serial.print(bytes_read);
            // Convert the bits to hex
            Serial.println("bytes read");

            if (data_read[bytes_read - 1] == '\r') data_read[bytes_read - 1] = 0;

            hilsim_reader = HILSIMPacket_init_zero;
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)(data_read), bytes_read);

            bool status = pb_decode(&stream, HILSIMPacket_fields, &hilsim_reader);

            if (!status) {
                Serial.print("Decoding failed: ");
                Serial.println(PB_GET_ERROR(&stream));
            } else {
                Serial.print("ax: ");
                Serial.println(hilsim_reader.imu_high_ax);
                Serial.print("ay: ");
                Serial.println(hilsim_reader.imu_high_ay);
                Serial.print("az: ");
                Serial.println(hilsim_reader.imu_high_az);
                Serial.print("Barom alt: ");
                Serial.println(hilsim_reader.barometer_altitude);
            }

            data_read[bytes_read] = 0;
        } else {
            Serial.println("Got nothing");
        }

        chThdSleepMilliseconds(1);
    }
}
#endif

/******************************************************************************/
/* TELEMETRY THREAD                                         */

#ifdef ENABLE_TELEMETRY
bool telemetry_buffering_start = false;

static THD_FUNCTION(telemetry_buffering_THD, arg) {
    telemetry_buffering_start = true;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### telemetry buffering thread entrance");
#endif
        tlm.bufferData();
        chThdSleepMilliseconds(80);
    }
}
#endif

/******************************************************************************/
/* TELEMETRY THREAD                                         */

#ifdef ENABLE_TELEMETRY
bool telemetry_sending_start = false;

static THD_FUNCTION(telemetry_sending_THD, arg) {
    telemetry_sending_start = true;

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
#endif

/******************************************************************************/
/* ROCKET FINITE STATE MACHINE THREAD                                         */

bool rocket_fsm_start = false;

static THD_FUNCTION(rocket_FSM_THD, arg) {
    rocket_fsm_start = true;

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
/* SENSOR FAST THREAD                                                          */

#ifdef ENABLE_SENSOR_FAST
bool sensor_fast_start = false;

static THD_FUNCTION(sensor_fast_THD, arg) {
    sensor_fast_start = true;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Sensor fast thread entrance");
#endif
#ifdef ENABLE_HILSIM_MODE

        barometer.update(hilsim_reader);
        magnetometer.update(hilsim_reader);
        gas.refresh();
        orientation.update(hilsim_reader);
        lowG.update(hilsim_reader);
        voltage.read();
        highG.update(hilsim_reader);

#else
        barometer.update();
        magnetometer.update();
        gas.refresh();
        orientation.update();
        lowG.update();
        voltage.read();
        highG.update();
#endif

        chThdSleepMilliseconds(6);
    }
}
#endif

/******************************************************************************/
/* GPS THREAD                                                                 */

#ifdef ENABLE_GPS
bool gps_start = false;

static THD_FUNCTION(gps_THD, arg) {
    gps_start = true;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### GPS thread entrance");
#endif
        gps.update();

        chThdSleepMilliseconds(190);  // Read the gps @ ~5 Hz
    }
}
#endif

/******************************************************************************/
/* KALMAN FILTER THREAD                                                       */

bool kalman_start = false;

static THD_FUNCTION(kalman_THD, arg) {
    kalman_start = true;

    kalmanFilter.Initialize();

    systime_t last = chVTGetSystemTime();
    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Kalman thread entrance");
#endif
        // Serial.println("entering tick");
        kalmanFilter.kfTickFunction(TIME_I2MS(chVTGetSystemTime() - last), 13.0);
        // Serial.println("exiting tick");
        // Serial.println(TIME_I2MS(chVTGetSystemTime() - last));
        last = chVTGetSystemTime();

        chThdSleepMilliseconds(50);
    }
}

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

bool servo_start = false;

static THD_FUNCTION(servo_THD, arg) {
    servo_start = true;

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
/* DATA LOGGER THREAD                                                   */

#ifdef ENABLE_SD
bool sd_start = false;

static THD_FUNCTION(dataLogger_THD, arg) {
    sd_start = true;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("Data Logging thread entrance");
#endif
        if (datalog_is_running) {
            sd_logger.update();
        }

        chThdSleepMilliseconds(6);
    }
}
#endif

/******************************************************************************/
/* BUZZER THREAD                                                   */

#ifdef ENABLE_BUZZER
bool buzzer_start = false;

static THD_FUNCTION(buzzer_THD, arg) {
    buzzer_start = true;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("Buzzer thread entrance");
#endif

        buzzer1.tick();

        chThdSleepMilliseconds(6);
    }
}
#endif

#define THREAD_WA 8192

#ifdef ENABLE_HILSIM_MODE
static THD_WORKING_AREA(hilsim_WA, THREAD_WA);
#endif
#ifdef ENABLE_GPS
static THD_WORKING_AREA(gps_WA, THREAD_WA);
#endif
static THD_WORKING_AREA(rocket_FSM_WA, THREAD_WA);
static THD_WORKING_AREA(servo_WA, THREAD_WA);
#ifdef ENABLE_SD
static THD_WORKING_AREA(dataLogger_WA, THREAD_WA);
#endif
#ifdef ENABLE_TELEMETRY
static THD_WORKING_AREA(telemetry_sending_WA, THREAD_WA);
static THD_WORKING_AREA(telemetry_buffering_WA, THREAD_WA);
#endif
static THD_WORKING_AREA(kalman_WA, THREAD_WA);
#ifdef ENABLE_BUZZER
static THD_WORKING_AREA(buzzer_WA, THREAD_WA);
#endif
#ifdef ENABLE_SENSOR_FAST
static THD_WORKING_AREA(sensor_fast_WA, THREAD_WA);
#endif

#undef THREAD_WA

#define START_THREAD(NAME) chThdCreateStatic(NAME##_WA, sizeof(NAME##_WA), NORMALPRIO + 1, NAME##_THD, nullptr)
#define CHECK_THREAD(NAME, SHORT)                         \
    do {                                                  \
        Serial.print(" " SHORT ": ");                     \
        Serial.print(NAME##_start ? "\u2713" : "\u2717"); \
        all_passed &= NAME##_start;                       \
    } while (0)

/**
 * @brief Starts all of the threads.
 */
[[noreturn]] void chSetup() {
#ifdef ENABLE_HILSIM_MODE
    START_THREAD(hilsim);
#endif
#ifdef ENABLE_TELEMETRY
    START_THREAD(telemetry_sending);
    START_THREAD(telemetry_buffering);
#endif
    START_THREAD(rocket_FSM);
#ifdef ENABLE_GPS
    START_THREAD(gps);
#endif
#ifdef ENABLE_SENSOR_FAST
    START_THREAD(sensor_fast);
#endif
    START_THREAD(servo);
#ifdef ENABLE_SD
    START_THREAD(dataLogger);
#endif
    START_THREAD(kalman);
#ifdef ENABLE_BUZZER
    START_THREAD(buzzer);
#endif

    bool all_passed;
    do {
        digitalWrite(LED_GREEN, HIGH);
        all_passed = true;

        Serial.print("Thread Starts:");
#ifdef ENABLE_TELEMETRY
        CHECK_THREAD(telemetry_sending, "TLMS");
        CHECK_THREAD(telemetry_buffering, "TLMB");
#endif
        CHECK_THREAD(rocket_fsm, "FSM");
#ifdef ENABLE_GPS
        CHECK_THREAD(gps, "GPS");
#endif
#ifdef ENABLE_SENSOR_FAST
        CHECK_THREAD(sensor_fast, "SF");
#endif
        CHECK_THREAD(servo, "SRV");
#ifdef ENABLE_SD
        CHECK_THREAD(sd, "SD");
#endif
        CHECK_THREAD(kalman, "KLMN");
#ifdef ENABLE_BUZZER
        CHECK_THREAD(buzzer, "BUZZ");
#endif
        Serial.println("");

        chThdSleepMilliseconds(200);
        digitalWrite(LED_GREEN, LOW);
        chThdSleepMilliseconds(200);
    } while (!all_passed);
    // buzzer1.init_sponge();
    buzzer1.init_sponge();
    // buzzer1.init_mario();
    while (true) {
        chThdYield();
    }
}

#undef CHECK_THREAD
#undef START_THREAD

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */

void setup() {
    Serial.begin(9600);
#ifdef WAIT_SERIAL
    // Stall until serial monitor opened
    while (!Serial)
        ;
#endif
    Serial.println("Starting SPI...");

#ifndef ENABLE_SILSIM_MODE
    SPI.begin();
    SPI.setMOSI(SPI_MOSI);
    SPI.setMISO(SPI_MISO);
    SPI.setSCK(SPI_SCK);

    SPI1.begin();
    SPI1.setMOSI(B2B_SPI_MOSI);
    SPI1.setMISO(B2B_SPI_MISO);
    SPI1.setSCK(B2B_SPI_SCK);

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

    Wire.setSCL(MAXM10S_SCL);
    Wire.setSDA(MAXM10S_SDA);
    Wire.begin();
#endif

#ifdef ENABLE_BAROMETER
    handleError(barometer.init());
#endif
#ifdef ENABLE_GAS
    handleError(gas.init());
#endif
#ifdef ENABLE_GPS
    handleError(gps.init());
#endif
#ifdef ENABLE_HIGH_G
    handleError(highG.init());
#endif
#ifdef ENABLE_LOW_G
    handleError(lowG.init());
#endif
#ifdef ENABLE_MAGNETOMETER
    handleError(magnetometer.init());
#endif
#ifdef ENABLE_ORIENTATION
    handleError(orientation.init());
#endif

#ifdef ENABLE_SD
    handleError(sd_logger.init());
#endif
#ifdef ENABLE_TELEMETRY
    handleError(tlm.init());
#endif

    Serial.println("chibios begin");
    chBegin(chSetup);
}

void loop() {
    // not used
}

#undef ENABLE_SENSOR_FAST
