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
 */

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "ActiveControl.h"
#include "FSMCollection.h"
#include "HistoryBufferFSM50.h"
#include "HistoryBufferFSM6.h"
#include "KalmanFSM.h"
#include "MS5611.h"  //Barometer library
#include "ServoControl.h"
#include "SparkFunLSM9DS1.h"                       //Low-G IMU Library
#include "SparkFun_Qwiic_KX13X.h"                  //High-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"  //GPS Library
#include "TimerFSM.h"
#include "dataLog.h"
#include "kalmanFilter.h"
#include "pins.h"
#include "rocketFSM.h"
#include "sensors.h"
#include "telemetry.h"
#include "global_data_hack.h"

SensorState global_state;

// DataLogBuffer datalogger_THD_vars;

// #define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

// Create a data struct to hold data from the sensors
sensorDataStruct_t sensorData;

QwiicKX134 highGimu;
LSM9DS1 lowGimu;
bool gps_connected;
SFE_UBLOX_GNSS gps;

MS5611 barometer{MS5611_CS};

PWMServo ac_servo;  // Servo that induces clockwise roll moment

// Create a struct that holds pointers to all the important objects needed by
// the threads
pointers sensor_pointers;

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
static THD_WORKING_AREA(data_ingest_WA, 8192);


static THD_FUNCTION(data_ingest_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;
    char buff[512];
    Serial.setTimeout(10000);
    Serial.print("begin");
    while(true){
        int read_count = Serial.readBytesUntil('\n', buff, 511);
        if(read_count > 0){
            if(buff[read_count-1] == '\r') buff[read_count-1] = 0;
            SensorState& s = global_state;
            sscanf(buff, "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
            &s.time_ms,
            &s.ax,
            &s.ay,
            &s.az,
            &s.gx,
            &s.gy,
            &s.gz,
            &s.mx,
            &s.my,
            &s.mz,
            &s.latitude,
            &s.longitude,
            &s.altitude,
            &s.satellite_count,
            &s.position_lock,
            &s.temperature,
            &s.pressure,
            &s.barometer_altitude,
            &s.highg_ax,
            &s.highg_ay,
            &s.highg_az,
            &s.rocket_state0,
            &s.rocket_state1,
            &s.rocket_state2,
            &s.rocket_state3,
            &s.flap_extension,
            &s.state_est_x,
            &s.state_est_vx,
            &s.state_est_ax,
            &s.state_est_apo,
            &s.battery_voltage);
            s.time_ch = TIME_MS2I(s.time_ms);
            buff[read_count] = 0;
            pointer_struct->sensorDataPointer->barometer_data.altitude = s.altitude;
            pointer_struct->sensorDataPointer->barometer_data.pressure = s.pressure;
            pointer_struct->sensorDataPointer->barometer_data.temperature = s.temperature;
            pointer_struct->sensorDataPointer->gps_data.altitude = s.altitude;
            pointer_struct->sensorDataPointer->gps_data.latitude = s.latitude;
            pointer_struct->sensorDataPointer->gps_data.longitude = s.longitude;
            pointer_struct->sensorDataPointer->gps_data.posLock = s.position_lock;
            pointer_struct->sensorDataPointer->gps_data.siv_count = s.satellite_count;
            pointer_struct->sensorDataPointer->voltage_data.v_battery = s.battery_voltage;
            pointer_struct->sensorDataPointer->lowG_data.ax = s.ax;
            pointer_struct->sensorDataPointer->lowG_data.ay = s.ay;
            pointer_struct->sensorDataPointer->lowG_data.az = s.az;
            pointer_struct->sensorDataPointer->lowG_data.mx = s.mx;
            pointer_struct->sensorDataPointer->lowG_data.my = s.my;
            pointer_struct->sensorDataPointer->lowG_data.mz = s.mz;
            pointer_struct->sensorDataPointer->lowG_data.gx = s.gx;
            pointer_struct->sensorDataPointer->lowG_data.gy = s.gy;
            pointer_struct->sensorDataPointer->lowG_data.gz = s.gz;
            pointer_struct->sensorDataPointer->highG_data.hg_ax = s.highg_ax;
            pointer_struct->sensorDataPointer->highG_data.hg_ay = s.highg_ay;
            pointer_struct->sensorDataPointer->highG_data.hg_az = s.highg_az;
            pointer_struct->sensorDataPointer->rocketState_data.rocketStates[0] = (RocketFSM::FSM_State)s.rocket_state0;
            pointer_struct->sensorDataPointer->rocketState_data.rocketStates[1] = (RocketFSM::FSM_State)s.rocket_state1;
            pointer_struct->sensorDataPointer->rocketState_data.rocketStates[2] = (RocketFSM::FSM_State)s.rocket_state2;
            pointer_struct->sensorDataPointer->rocketState_data.rocketStates[3] = (RocketFSM::FSM_State)s.rocket_state3;
            pointer_struct->sensorDataPointer->state_data.state_x = s.state_est_x;
            pointer_struct->sensorDataPointer->state_data.state_vx = s.state_est_vx;
            pointer_struct->sensorDataPointer->state_data.state_ax = s.state_est_ax;
            pointer_struct->sensorDataPointer->state_data.state_apo = s.state_est_apo;
        }
        chThdSleepMilliseconds(1);
    }
}
/******************************************************************************/
/* TELEMETRY THREAD                                         */

static THD_FUNCTION(telemetry_buffering_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    while (pointer_struct->telemetry == nullptr) chThdSleepMilliseconds(50);

    Telemetry &tlm = *pointer_struct->telemetry;
    while (true) {
        tlm.buffer_data(sensorData);
        chThdSleepMilliseconds(80);
    }
}

/******************************************************************************/
/* TELEMETRY THREAD                                         */

static THD_FUNCTION(telemetry_sending_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    // Telemetry& tlm = *pointer_struct->telemetry;
    Telemetry tlm;
    pointer_struct->telemetry = &tlm;
    while (true) {
        tlm.transmit(sensorData);
        pointer_struct->abort = tlm.abort;
        chThdSleepMilliseconds(200);
        // transmit has a sleep in it
    }
}

/******************************************************************************/
/* ROCKET FINITE STATE MACHINE THREAD                                         */

static THD_FUNCTION(rocket_FSM, arg) {
    pointers *pointer_struct = (struct pointers *)arg;

    // Implement RocketFSM class and instantiate it here
    // Refer to TemplateFSM for an example
    TimerFSM timer_fsm(pointer_struct);
    HistoryBufferFSM50 history_buffer_fsm_50(pointer_struct);
    HistoryBufferFSM6 history_buffer_fsm_6(pointer_struct);
    KalmanFSM kalman_fsm(pointer_struct);

    // Add FSM pointer to array of FSMs to be updated
    RocketFSM *fsm_array[] = {&timer_fsm, &history_buffer_fsm_50, &history_buffer_fsm_6, &kalman_fsm};

    // Pass array of FSMs to FSMCollection along with number of FSMs in use
    FSMCollection<4> fsms(fsm_array);

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Rocket FSM thread entrance");
#endif

        chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_rocket_state);

        fsms.tick();

        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_rocket_state);

        rocketStateData<4> fsm_state;
        fsms.getStates(fsm_state);
        // for (size_t i = 2; i < 3; i++) {
        //     Serial.print(i);
        //     Serial.print(": ");
        //     Serial.println((int) fsm_state.rocketStates[i]);
        // }
        pointer_struct->sensorDataPointer->rocketState_data = fsm_state;
        pointer_struct->dataloggerTHDVarsPointer.pushRocketStateFifo(&fsm_state);

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
    BarometerData *barometer_data = &pointer_struct->sensorDataPointer->barometer_data;

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

        chThdSleepMilliseconds(20);  // highg reads at 50 hz
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
    systime_t last = global_state.time_ch;
    while (true) {
        // #ifdef THREAD_DEBUG
        //     Serial.println("In Kalman");
        // #endif
        KF.kfTickFunction(TIME_I2MS(global_state.time_ch - last), 13.0);
        last = global_state.time_ch;
        // Serial.println("kalman");
        // Serial.println(pointer_struct->sensorDataPointer->state_data.state_x);

        chThdSleepMilliseconds(50);
    }
}

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

static THD_FUNCTION(servo_THD, arg) {
    struct pointers *pointer_struct = (struct pointers *)arg;

    Controller ac(pointer_struct, &ac_servo);

    while (true) {
#ifdef THREAD_DEBUG
        Serial.println("### Servo thread entrance");
#endif

        ac.ctrlTickFunction(pointer_struct);

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

        chThdSleepMilliseconds(6);  // Set equal sleep time as the other threads, can change
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
    chThdCreateStatic(data_ingest_WA, sizeof(data_ingest_WA), NORMALPRIO + 1, data_ingest_THD, &sensor_pointers);
    // chThdCreateStatic(telemetry_sending_WA, sizeof(telemetry_sending_WA), NORMALPRIO + 1, telemetry_sending_THD,
    //                   &sensor_pointers);
    // chThdCreateStatic(telemetry_buffering_WA, sizeof(telemetry_buffering_WA), NORMALPRIO + 1, telemetry_buffering_THD,
    //                   &sensor_pointers);
    // chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO + 1, rocket_FSM, &sensor_pointers);
    // chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO + 1, gps_THD, &sensor_pointers);
    // chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO + 1, lowgIMU_THD, &sensor_pointers);
    // chThdCreateStatic(barometer_WA, sizeof(barometer_WA), NORMALPRIO + 1, barometer_THD, &sensor_pointers);
    // chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO + 1, highgIMU_THD, &sensor_pointers);
    chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO + 1, servo_THD, &sensor_pointers);
    // chThdCreateStatic(dataLogger_WA, sizeof(dataLogger_WA), NORMALPRIO + 1, dataLogger_THD, &sensor_pointers);
    // chThdCreateStatic(voltage_WA, sizeof(voltage_WA), NORMALPRIO + 1, voltage_THD, &sensor_pointers);
    // chThdCreateStatic(kalman_WA, sizeof(kalman_WA), NORMALPRIO + 1, kalman_THD, &sensor_pointers);

    while (true)
        ;
}

/**
 * @brief Handles all configuration necessary before the threads start.
 *
 */

void setup() {
    Serial.begin(9600);
    while(!Serial);
    // pinMode(LED_BLUE, OUTPUT);
    // pinMode(LED_RED, OUTPUT);
    // pinMode(LED_ORANGE, OUTPUT);
    // pinMode(LED_GREEN, OUTPUT);

    // pinMode(LSM9DS1_AG_CS, OUTPUT);
    // digitalWrite(LSM9DS1_AG_CS, HIGH);
    // pinMode(LSM9DS1_M_CS, OUTPUT);
    // digitalWrite(LSM9DS1_M_CS, HIGH);
    // pinMode(ZOEM8Q0_CS, OUTPUT);
    // digitalWrite(ZOEM8Q0_CS, HIGH);
    // pinMode(MS5611_CS, OUTPUT);
    // digitalWrite(MS5611_CS, HIGH);
    // pinMode(KX134_CS, OUTPUT);
    // digitalWrite(KX134_CS, HIGH);
    // pinMode(H3LIS331DL_CS, OUTPUT);
    // digitalWrite(H3LIS331DL_CS, HIGH);
    // pinMode(RFM95_CS, OUTPUT);
    // digitalWrite(RFM95_CS, HIGH);

    // pinMode(LSM9DS1_AG_CS, OUTPUT);
    // pinMode(LSM9DS1_M_CS, OUTPUT);
    // pinMode(ZOEM8Q0_CS, OUTPUT);
    // pinMode(MS5611_CS, OUTPUT);

    // digitalWrite(LSM9DS1_AG_CS, HIGH);
    // digitalWrite(LSM9DS1_M_CS, HIGH);
    // digitalWrite(ZOEM8Q0_CS, HIGH);
    // digitalWrite(MS5611_CS, HIGH);

    sensor_pointers.lowGimuPointer = &lowGimu;
    sensor_pointers.highGimuPointer = &highGimu;
    sensor_pointers.barometerPointer = &barometer;
    sensor_pointers.GPSPointer = &gps;
    sensor_pointers.sensorDataPointer = &sensorData;
    sensor_pointers.abort = false;

    // SPI.begin();
    // SPI1.setMISO(39);

    // if (!highGimu.beginSPICore(KX134_CS, 1000000, SPI)) {
    //     Serial.println("Failed to communicate with KX134. Stalling Program");
    //     digitalWrite(LED_RED, HIGH);
    //     while (true)
    //         ;
    // }

    // if (!highGimu.initialize(DEFAULT_SETTINGS)) {
    //     Serial.println("Could not initialize KX134. Stalling Program");
    //     digitalWrite(LED_BLUE, HIGH);
    //     while (true)
    //         ;
    // }

    // highGimu.setRange(3);  // set range to 3 = 64 g range
    // // lowGimu setup
    // if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) ==
    //     false)  // note, we need to sent this our CS pins (defined above)
    // {
    //     digitalWrite(LED_ORANGE, HIGH);
    //     Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
    //     while (true)
    //         ;
    // }

    // // Initialize barometer
    // barometer.init();

    // // GPS Setup
    // SPI1.begin();
    // digitalWrite(LED_RED, HIGH);
    // digitalWrite(LED_ORANGE, HIGH);

    // if (!gps.begin(SPI1, ZOEM8Q0_CS, 4000000)) {
    //     Serial.println("Failed to communicate with ZOEM8Q0 gps. Stalling Program");
    //     while (true)
    //         ;
    // } else {
    // }
    // digitalWrite(LED_RED, LOW);
    // digitalWrite(LED_ORANGE, LOW);

    // gps.setPortOutput(COM_PORT_SPI,
    //                   COM_TYPE_UBX);                 // Set the SPI port to output UBX only
    //                                                  // (turn off NMEA noise)
    // gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save (only) the communications port settings
    //                                                  // to flash and BBR
    // gps.setNavigationFrequency(5);                   // set sampling rate to 5hz

    // // Telemetry tlm;
    // // sensor_pointers.telemetry = &tlm;
    // // SD Card Setup
    // if (SD.begin(BUILTIN_SDCARD)) {
    //     char file_extension[6] = ".dat";

    //     char data_name[16] = "data";
    //     // Initialize SD card
    //     sensor_pointers.dataloggerTHDVarsPointer.dataFile =
    //         SD.open(sd_file_namer(data_name, file_extension), O_CREAT | O_WRITE | O_TRUNC);

    //     // print header to file on sd card that lists each variable that is
    //     // logged
    //     // auto written =
    //     // sensor_pointers.dataloggerTHDVarsPointer.dataFile.println(
    //     //     "binary logging of sensor_data_t");
    //     sensor_pointers.dataloggerTHDVarsPointer.dataFile.flush();
    //     //
    //     Serial.println(sensor_pointers.dataloggerTHDVarsPointer.dataFile.name());
    // } else {
    //     digitalWrite(LED_RED, HIGH);
    //     digitalWrite(LED_ORANGE, HIGH);
    //     digitalWrite(LED_BLUE, HIGH);
    //     Serial.println("SD Begin Failed. Stalling Program");
    //     while (true) {
    //         digitalWrite(LED_RED, HIGH);
    //         delay(100);
    //         digitalWrite(LED_RED, LOW);
    //         delay(100);
    //     }
    // }

    // digitalWrite(LED_ORANGE, HIGH);
    // digitalWrite(LED_BLUE, HIGH);
    // // Servo Setup
    ac_servo.attach(AC_SERVO_PIN, 770, 2250);
    chBegin(chSetup);
    while (true)
        ;
}

void loop() {
    // not used
}
