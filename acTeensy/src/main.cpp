/* main.cpp
 *   ______  ___     ___    ____        __  _____ __   ___
 *  /_  __/ / _ |   / _ \  / __/ ____  /  |/  / //_/  <  /
 *   / /   / __ |  / , _/ _\ \  /___/ / /|_/ / ,<     / /
 *  /_/   /_/ |_| /_/|_| /___/       /_/  /_/_/|_|   /_/
 *
 * Active Control Teensy Program
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Anshuk Chigullapalli
 * Josh Blustein
 * Ayberk Yaraneri
 * David Robbins
 * Matt Taylor
 * James Bayus
 * Ben Olaivar
 * TODO: add missing names if any
 */

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "SparkFunLSM9DS1.h"
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "thresholds.h"
#include "pins.h"

// #define USB_SERIAL_BEGIN
// #define THREAD_DEBUG
// #define SENSOR_DEBUG

//TODO: SWAP THESE IN HYBRID TEENSY!!!!!!!!!!
#define TT_SEND_PIN 38
#define TT_RECEIVE_PIN 39

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;

//create servo objects for the ball valve servos
PWMServo ballValve1;
PWMServo ballValve2;

//create imu object
LSM9DS1 imu;

//create file object for SD card
File dataFile;

int currentAngle; //Current angle of the servos.
int initialAngle = 0; //Initial angle of servos. This may not be zero.
float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float old_alt; //Altitude from previous cycle
float roll_rate; //angular velocity in the z direction
float g = 9.81; //Acceleration due to gravity in m/s
float latitude; //current latitude from gps
float longitude; //current longitude from gps
float Kp; //Proportionality constant for roll control PID
float rr_thresh; //Maximum roll rate that we ignore.
float roll_off_alt; //Altitude when roll control stops and we do active drag
float des_alt; //Final altitude goal
float buffer; //Buffer for the active drag
float m; //Mass of the rocket
pressureData hybridData;

fsm_struct fsm_states;

FSM_State rocketState;

dataStruct_t sensorData;

bool teensy_fail = false; //Status of the other teensy, initialized to false. False means active, true means failed.

bool ballValveOpen = false; //True if ball valve is open, false if closed.

uint8_t BB_data[26] = {0x49, 0x53, 0x53};

//------------------------------------------------------------------------------
// dataThread - collects data from sensors, stored to SD card.
static THD_WORKING_AREA(dataThread_WA, 256);
thread_t *dataThread_Pointer;

static THD_FUNCTION(dataThread, arg) {
    (void)arg;

    systime_t last = chVTGetSystemTime();
    systime_t duration = 0;

    while (true) {

#ifdef THREAD_DEBUG
        Serial.println("### data thread entrance");
#endif

        digitalWrite(LED_BLUE, HIGH);

        // TODO - Acquire lock on rocketState and sensorData!!!!

        imu.readAccel(); //update IMU accelerometer data
        imu.readGyro();
        imu.readMag();

        sensorData.ax = imu.az;
        sensorData.ay = imu.ax;
        sensorData.az = -imu.ay;
        sensorData.gx = imu.gx;
        sensorData.gy = imu.gy;
        sensorData.gz = imu.gz;
        sensorData.mx = imu.mx;
        sensorData.my = imu.my;
        sensorData.mz = imu.mz;
        // sensorData.pt1 = ptConversion(analogRead(HYBRID_PT_1_PIN));
        // sensorData.pt2 = ptConversion(analogRead(HYBRID_PT_2_PIN));
        // sensorData.pt2 = ptConversion(analogRead(HYBRID_PT_2_PIN));
        sensorData.timeStamp = chVTGetSystemTime();

#ifdef SENSOR_DEBUG
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
        Serial.println(sensorData.mz);
#endif

        logData(&dataFile, &sensorData, rocketState);

        digitalWrite(LED_BLUE, LOW);

        // Just for verifying DAQ frequency
        // duration = chVTGetSystemTime() - last;
        // Serial.print("\t\t\t\t\t DURATION: ");
        // Serial.println(duration);
        // last = chVTGetSystemTime();

        chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
    }

}

//------------------------------------------------------------------------------
// ttReceive - Thread for recieving other teensy's heartbeat
static THD_WORKING_AREA(ttReceive_WA, 32);
thread_t *ttReceive_Pointer;

static THD_FUNCTION(ttReceive_THD, arg){
    while(true){

#ifdef THREAD_DEBUG
        Serial.println("### ttReceive thread entrance");
#endif

        // // checks every 250 millisecond if the other teensy has NOT responded
        // while(!(pulseIn(TT_Receive_PIN, HIGH, 250000) > 0)) {
        //     //other teensy has failed...activate other threads
        //     teensy_fail = true;
        // }

        chThdSleepMilliseconds(50);
    }
}

//------------------------------------------------------------------------------
// ttSend - sends heartbeat for other teensy to listen to every 50 milliseconds
static THD_WORKING_AREA(ttSend_WA, 32);
thread_t *ttSend_Pointer;

static THD_FUNCTION(ttSend_THD, arg){
    while(true){

#ifdef THREAD_DEBUG
        Serial.println("### ttSend thread entrance");
#endif

        digitalWrite(TT_SEND_PIN, HIGH);
        chThdSleepMilliseconds(50);
        digitalWrite(TT_SEND_PIN, LOW);
        chThdSleepMilliseconds(50);
    }
}

//------------------------------------------------------------------------------
// bbComm - sends information to BeagleBone at 5 Hz
static THD_WORKING_AREA(bbComm_WA, 64);
thread_t *bbComm_Pointer;

static THD_FUNCTION(bbComm_THD, arg) {
    (void)arg;

    Serial1.begin(115200);

    while(true) {

#ifdef THREAD_DEBUG
        Serial.println("### bbComm thread entrance");
#endif

        digitalWrite(LED_WHITE, HIGH);
        //
        // int i = 3;
        //
        // uint8_t* data = (uint8_t*) &sensorData;
        //
        // for (; i < 3 + sizeof(sensorData); ++i) {
        //     BB_data[i] = data;
        //     ++data;
        // }
        //
        // BB_data[25] = rocketState;
        //
        // Serial1.write(BB_data, 25);
        //
        digitalWrite(LED_WHITE, LOW);

        chThdSleepMilliseconds(100);
    }
}

//------------------------------------------------------------------------------
// servoThread - Thread for actuating servos for active control, updates at 20Hz
static THD_WORKING_AREA(servoThread_WA, 32);
thread_t *servoThread_Pointer;

static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bottom. Anshuk: I don't get why this is here.
    while(true) {

#ifdef THREAD_DEBUG
        Serial.println("### servoThread thread entrance");
#endif

        /* THOU SHALL NOT END THE PROGRAM */
        // if(servo1.attached() == false || servo2.attached() == false || servo3.attached() == false || servo4.attached() == false) {
        //     delay(TERMINATE); //basically end the program. Anshuk: Maybe we should just end this thread rather than the whole program
        // }

        //To determine whether the rocket should use roll/drag control or not. Only done in the coast phase
        if(rocketState == STATE_COAST) {
            //record time using millis
            if((roll_rate > rr_thresh || roll_rate < -rr_thresh) && altitude < roll_off_alt)//If the rocket is rolling below a certain altitude..
            {
                //...reduce the roll
                //Proportional controller to reduce the roll of the rocket. Kp is the proportional gain.
                float roll_err = -roll_rate; //The desired roll rate is 0, so the error = 0-roll
                float theta = Kp*roll_err; //Make sure the Kp accounts for radian-degree conversion!

                //Make sure this block behaves as expected and outputs an angle of theta
                servo1.write(constrain(initialAngle+theta,initialAngle-15,initialAngle+15));
                servo2.write(constrain(initialAngle+theta,initialAngle-15,initialAngle+15));
                servo3.write(constrain(initialAngle+theta,initialAngle-15,initialAngle+15));
            }

            else {
                //implement active drag. This needs to be tested
                //control_vel = f(altitude)
                float native_drag; //Need to calculate this using velocity and CD of rocket without flaps
                float control_vel = sqrt(2*(g+(native_drag/m)*(des_alt-altitude))); //From v^2 - u^2 = 2*a*s equation. THe final velocity is zero,...
                //...u is the desired velocity considering flap drag isn't applied throughout
                if (altitude > des_alt) {
                    //flaps are vertical
                    servo1.write(initialAngle);
                    servo2.write(initialAngle);
                    servo3.write(initialAngle);
                }
                else {
                    if (velocity > control_vel*(1+buffer/100))
                    {
                        //flaps are deployed
                        servo1.write(initialAngle);
                        servo2.write(initialAngle);
                        servo3.write(initialAngle);
                    }
                    else {
                        //go to vertical
                        servo1.write(initialAngle);
                        servo2.write(initialAngle);
                        servo3.write(initialAngle);
                    }
                }
            }
        }

        chThdSleepMilliseconds(50);
    }
}

//------------------------------------------------------------------------------
// ballValve- thread that controls the ball valve servos for the hybrid engine.
// Hybrid teensy threads that need to be ASLEEP if hybrid teensy is working
static THD_WORKING_AREA(ballValve_WA, 32);
thread_t *ballValve_Pointer;

static THD_FUNCTION(ballValve_THD, arg){
    while(true){

#ifdef THREAD_DEBUG
        Serial.println("### ballValve thread entrance");
#endif

        while(teensy_fail == true) {
            //if fail safe switch is active, then don't active this thread
            //Anshuk: Once FSM is developed, make conditionals below more involved
            if(ballValveOpen == false){
                ballValve1.write(180);
                ballValve2.write(180);
                digitalWrite(LED_RED, HIGH);
            }
            else if(ballValveOpen == true){
                ballValve1.write(0);
                ballValve2.write(0);
                digitalWrite(LED_RED, LOW);
            }
        }

        chThdSleepMilliseconds(100);
    }
}

//------------------------------------------------------------------------------
// ballValve- Thread for the rocket state transition logic
static THD_WORKING_AREA(rocket_FSM_WA, 32);
thread_t *rocket_FSM_Pointer;

static THD_FUNCTION(rocket_FSM, arg) {
    (void)arg;

    while(true) {

#ifdef THREAD_DEBUG
        Serial.println("### rocket_FSM thread entrance");
#endif
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO - Acquire lock on data struct!

        switch (rocketState) {
            case STATE_INIT:
                // TODO
            break;

            case STATE_IDLE:

                // If high acceleration is observed in z direction...
                if(sensorData.az > launch_az_thresh) {
                    fsm_states.launch_time = chVTGetSystemTime();
                    rocketState = STATE_LAUNCH_DETECT;
                }

            break;

            case STATE_LAUNCH_DETECT:

                //If the acceleration was too brief, go back to IDLE
                if (sensorData.az < launch_az_thresh) {
                    rocketState = STATE_IDLE;
                    break;
                }

                // measure the length of the burn time (for hysteresis)
                fsm_states.burn_timer =
                    chVTGetSystemTime() - fsm_states.launch_time;

                // If the acceleration lasts long enough, boost is detected
                if (fsm_states.burn_timer > launch_time_thresh) {
                    rocketState = STATE_BOOST;
                    digitalWrite(LED_ORANGE, HIGH);
                }

            break;

            case STATE_BOOST:

            // If low acceleration in the Z direction...
            if (sensorData.az < coast_thresh) {
                fsm_states.burnout_time = chVTGetSystemTime();
                rocketState = STATE_BURNOUT_DETECT;
            }

            break;

            case STATE_BURNOUT_DETECT:

                //If the low acceleration was too brief, go back to BOOST
                if (sensorData.az > coast_thresh) {
                    rocketState = STATE_BOOST;
                    break;
                }

                // measure the length of the coast time (for hysteresis)
                fsm_states.coast_timer =
                    chVTGetSystemTime() - fsm_states.burnout_time;

                // If the low acceleration lasts long enough, coast is detected
                if (fsm_states.coast_timer > coast_time_thresh) {
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

        chThdSleepMilliseconds(10); // FSM runs at 100 Hz
    }
}

//------------------------------------------------------------------------------
// chSetup - let there be threads
void chSetup() {

    ballValve1.attach(BALL_VALVE_1_PIN);
    ballValve2.attach(BALL_VALVE_2_PIN);
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    // testServos();

    servoThread_Pointer = chThdCreateStatic(servoThread_WA, sizeof(servoThread_WA), NORMALPRIO, servoThread, NULL);
    dataThread_Pointer = chThdCreateStatic(dataThread_WA, sizeof(dataThread_WA), NORMALPRIO, dataThread,NULL);
    // bbComm_Pointer = chThdCreateStatic(bbComm_WA, sizeof(bbComm_WA), NORMALPRIO, bbComm_THD, NULL);
    ballValve_Pointer = chThdCreateStatic(ballValve_WA, sizeof(ballValve_WA), NORMALPRIO, ballValve_THD, NULL);
    rocket_FSM_Pointer = chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, NULL);
    // ttReceive_Pointer = chThdCreateStatic(ttReceive_WA, sizeof(ttReceive_WA), NORMALPRIO, ttReceive_THD, NULL);
    // ttSend_Pointer = chThdCreateStatic(ttSend_WA, sizeof(ttSend_WA), NORMALPRIO, ttSend_THD, NULL);

}

void setup() {


    pinMode(LED_WHITE, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    digitalWrite(LED_WHITE, HIGH);

#ifdef USB_SERIAL_BEGIN
    Serial.begin(9600);
    while (!Serial) {}
#endif

    //set initial rocket state to idle
    rocketState = STATE_IDLE;

    //IMU Sensor Setup
    if(!imu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS)){
       digitalWrite(LED_RED, HIGH);
    }

    //SD Card Setup
    if(SD.begin(BUILTIN_SDCARD)){
        init_dataLog(&dataFile);
    }
    else {
#ifdef THREAD_DEBUG
        Serial.println("SD Begin Failed");
#endif
        digitalWrite(LED_RED, HIGH);
    }

    pinMode(TT_SEND_PIN, OUTPUT);
    pinMode(TT_RECEIVE_PIN, INPUT);

    Serial.println("starting chibiOS");
    //Initialize and start ChibiOS (Technically the first thread)
    chBegin(chSetup);

    while(true) {}

}

void loop() {

    chThdSleepMilliseconds(1000);
}
