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
#include "SparkFun_Qwiic_KX13X.h"  //High-G IMU Library
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
#include "kalmanFilter.h"

// DataLogBuffer datalogger_THD_vars;

// #define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

// Create a data struct to hold data from the sensors
sensorDataStruct_t sensorData;

FSM_State rocketState = STATE_INIT;

QwiicKX134 highGimu;
LSM9DS1 lowGimu;
bool gps_connected;
SFE_UBLOX_GNSS gps;

MS5611 barometer{MS5611_CS};

PWMServo servo_cw;   // Servo that induces clockwise roll moment

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
static THD_WORKING_AREA(voltage_WA, 8192);
static THD_WORKING_AREA(telemetry_WA, 8192);
static THD_WORKING_AREA(kalman_WA, 8192);

/******************************************************************************/
/* TELEMETRY THREAD                                         */

static THD_FUNCTION(telemetry_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;
    
    // int packetnum = 0;
    Telemetry tlm;
    while(true) {
        tlm.transmit(sensorData);
        pointer_struct->abort = tlm.abort;
        pointer_struct->testing_flaps = tlm.testing;
        chThdSleepMilliseconds(200);
        //transmit has a sleep in it
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

        chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_rocket_state);
        stateMachine.tickFSM();
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_rocket_state);


        chThdSleepMilliseconds(6);  // FSM runs at 100 Hz
    }
}

/******************************************************************************/
/* LOW G IMU THREAD                                                           */

static THD_FUNCTION(lowgIMU_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;
    LSM9DS1 *lsm = pointer_struct->lowGimuPointer;
    DataLogBuffer *data_log_buffer = &pointer_struct->dataloggerTHDVarsPointer;
    LowGData *lowG_Data = &pointer_struct->sensorDataPointer->lowG_data;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Low G IMU thread entrance");
#endif

        chMtxLock(&data_log_buffer->dataMutex_lowG);
        lowGimuTickFunction(lsm, data_log_buffer, lowG_Data);
        chMtxUnlock(&data_log_buffer->dataMutex_lowG);

        chThdSleepMilliseconds(6);
    }
}

/******************************************************************************/
/* BAROMETER THREAD                                                           */

static THD_FUNCTION(barometer_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    MS5611 *barometer = pointer_struct->barometerPointer;
    DataLogBuffer *data_log_buffer = &pointer_struct->dataloggerTHDVarsPointer;
    BarometerData *barometer_data =
        &pointer_struct->sensorDataPointer->barometer_data;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Barometer thread entrance");
#endif

        chMtxLock(&data_log_buffer->dataMutex_barometer);
        barometerTickFunction(barometer, data_log_buffer, barometer_data);
        chMtxUnlock(&data_log_buffer->dataMutex_barometer);


        chThdSleepMilliseconds(6);
    }
}

/******************************************************************************/
/* HIGH G IMU THREAD                                                          */

static THD_FUNCTION(highgIMU_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    QwiicKX134 *highG = pointer_struct->highGimuPointer;
    DataLogBuffer *data_log_buffer = &pointer_struct->dataloggerTHDVarsPointer;
    HighGData *highg_data = &pointer_struct->sensorDataPointer->highG_data;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### High G IMU thread entrance");
#endif
        chMtxLock(&data_log_buffer->dataMutex_highG);
        highGimuTickFunction(highG, data_log_buffer, highg_data);
        chMtxUnlock(&data_log_buffer->dataMutex_highG);

        chThdSleepMilliseconds(20); //highg reads at 50 hz
    }
}

/******************************************************************************/
/* GPS THREAD                                                                 */

static THD_FUNCTION(gps_THD, arg) {
    // Load outside variables into the function
    struct pointers *pointer_struct = (struct pointers *)arg;

    SFE_UBLOX_GNSS *gps = pointer_struct->GPSPointer;
    DataLogBuffer *data_log_buffer = &pointer_struct->dataloggerTHDVarsPointer;
    GpsData *gps_data = &pointer_struct->sensorDataPointer->gps_data;

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### GPS thread entrance");
#endif
        chMtxLock(&data_log_buffer->dataMutex_GPS);
        gpsTickFunction(gps, data_log_buffer, gps_data);
        chMtxUnlock(&data_log_buffer->dataMutex_GPS);

#ifdef THREAD_DEBUG
        Serial.println("### GPS thread exit");
#endif

        chThdSleepMilliseconds(190);  // Read the gps @ ~5 Hz
    }
}




/******************************************************************************/
/* KALMAN FILTER THREAD                                                       */

static THD_FUNCTION(kalman_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;
    KalmanFilter KF(pointer_struct);
    KF.Initialize();

    while(true){
        KF.kfTickFunction();

        chThdSleepMilliseconds(50);
    }


}

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

static THD_FUNCTION(servo_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    Controller ac(pointer_struct, &servo_cw);

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Servo thread entrance");
#endif

        ac.ctrlTickFunction();

        chThdSleepMilliseconds(6);  // FSM runs at 100 Hz
    }
}

/******************************************************************************/
/* MPU COMMUNICATION THREAD                                                   */

static THD_FUNCTION(voltage_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    DataLogBuffer *data_log_buffer = &pointer_struct->dataloggerTHDVarsPointer;
    VoltageData *volt_data = &pointer_struct->sensorDataPointer->voltage_data;

    VoltageSensor volt_sensor{Serial1};

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### voltage thread entrance");
#endif
        chMtxLock(&data_log_buffer->dataMutex_voltage);
        voltageTickFunction(&volt_sensor, data_log_buffer, volt_data);
        chMtxUnlock(&data_log_buffer->dataMutex_voltage);

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
    chThdCreateStatic(telemetry_WA, sizeof(telemetry_WA), NORMALPRIO + 1,
                      telemetry_THD, &sensor_pointers);
    chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO + 1,
                      rocket_FSM, &sensor_pointers);
    chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO + 1, gps_THD,
                      &sensor_pointers);
    chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO + 1,
                      lowgIMU_THD, &sensor_pointers);
    chThdCreateStatic(barometer_WA, sizeof(barometer_WA), NORMALPRIO + 1,
                      barometer_THD, &sensor_pointers);
    chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO + 1,
                      highgIMU_THD, &sensor_pointers);
    chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO + 1, 
                      servo_THD, &sensor_pointers);
    chThdCreateStatic(dataLogger_WA, sizeof(dataLogger_WA), NORMALPRIO + 1,
                      dataLogger_THD, &sensor_pointers);
    chThdCreateStatic(voltage_WA, sizeof(voltage_WA), NORMALPRIO + 1,
                      voltage_THD, &sensor_pointers);
    chThdCreateStatic(kalman_WA, sizeof(kalman_WA), NORMALPRIO + 1,
                      kalman_THD, &sensor_pointers);

    while (true)
        ;
}

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */

void setup() {
    Serial.begin(9600);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

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

    pinMode(LSM9DS1_AG_CS, OUTPUT);
    pinMode(LSM9DS1_M_CS, OUTPUT);
    pinMode(ZOEM8Q0_CS, OUTPUT);
    pinMode(MS5611_CS, OUTPUT);

    digitalWrite(LSM9DS1_AG_CS, HIGH);
    digitalWrite(LSM9DS1_M_CS, HIGH);
    digitalWrite(ZOEM8Q0_CS, HIGH);
    digitalWrite(MS5611_CS, HIGH);

    sensor_pointers.lowGimuPointer = &lowGimu;
    sensor_pointers.highGimuPointer = &highGimu;
    sensor_pointers.barometerPointer = &barometer;
    sensor_pointers.GPSPointer = &gps;
    sensor_pointers.sensorDataPointer = &sensorData;
    sensor_pointers.abort = false;
    sensor_pointers.testing_flaps = false;

    SPI.begin();
    SPI1.setMISO(39);

    if(!highGimu.beginSPICore(KX134_CS, 1000000, SPI)){
        Serial.println("Failed to communicate with KX134. Stalling Program");
        digitalWrite(LED_RED, HIGH);
        while (true)
            ;
        
    }

    if(!highGimu.initialize(DEFAULT_SETTINGS)){
        Serial.println("Could not initialize KX134. Stalling Program");
        digitalWrite(LED_BLUE, HIGH);
        while (true)
            ;
    }

    highGimu.setRange(3); // set range to 3 = 64 g range
    // lowGimu setup
    if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) ==
        false)  // note, we need to sent this our CS pins (defined above)
    {

        digitalWrite(LED_ORANGE, HIGH);
        Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
        while (true)
            ;
    }

    // Initialize barometer
    barometer.init();

    
    




    // GPS Setup
    SPI1.begin();
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_ORANGE, HIGH);

    if (!gps.begin(SPI1, ZOEM8Q0_CS, 4000000)) {
        Serial.println(
            "Failed to communicate with ZOEM8Q0 gps. Stalling Program");
        while (true)
            ;
    } else {

    }
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_ORANGE, LOW);



    gps.setPortOutput(COM_PORT_SPI,
                      COM_TYPE_UBX);  // Set the SPI port to output UBX only
                                      // (turn off NMEA noise)
    gps.saveConfigSelective(
        VAL_CFG_SUBSEC_IOPORT);  // Save (only) the communications port settings
                                 // to flash and BBR
    gps.setNavigationFrequency(5);  // set sampling rate to 10hz

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
            "binary logging of sensor_data_t");
        sensor_pointers.dataloggerTHDVarsPointer.dataFile.flush();
        // Serial.println(sensor_pointers.dataloggerTHDVarsPointer.dataFile.name());
    } else {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_ORANGE, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        Serial.println("SD Begin Failed. Stalling Program");
        while (true) {
            digitalWrite(LED_RED, HIGH);
            delay(100);
            digitalWrite(LED_RED, LOW);
            delay(100);
        }
    }

    digitalWrite(LED_ORANGE, HIGH);
    digitalWrite(LED_BLUE, HIGH);    
    // Servo Setup
    servo_cw.attach(SERVO_CW_PIN, 770, 2250);
    Serial.println("chibios begin");
    chBegin(chSetup);
    while (true)
        ;
}

void loop() {
    // not used
}
