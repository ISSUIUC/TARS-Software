#include <Arduino.h>
#include <ChRt.h>
#include <ch.h>
#include <unity.h>
#define class struct
#include "ZOEM8Q0.hpp"
#undef class

#include "rocketFSM.h"



void initial_state(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;

    rocketFSM fsm{&sensor_pointers};
    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_INIT);
    fsm.tickFSM();
}

void gps_lock_detect(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    ZOEM8Q0 gps;
    sensor_pointers.GPSPointer = &gps;
    gps.position_lock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_IDLE);
}


void launch_detect(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    ZOEM8Q0 gps;
    sensor_pointers.GPSPointer = &gps;
    gps.position_lock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->state_data.state_az = 60;
    fsm.tickFSM();
    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_LAUNCH_DETECT);
}

void boost_detect(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    ZOEM8Q0 gps;
    sensor_pointers.GPSPointer = &gps;
    gps.position_lock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->state_data.state_az = 60;
    for(int i = 0; i < 100; i++){
        fsm.tickFSM();
        chThdSleep(10);
    }
   
    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_BOOST);
}


void burnout_detect(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    ZOEM8Q0 gps;
    sensor_pointers.GPSPointer = &gps;
    gps.position_lock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->state_data.state_az = 60;
    for(int i = 0; i < 100; i++){
        fsm.tickFSM();
        chThdSleep(10);
    }
   
    sensor_pointers.sensorDataPointer->state_data.state_az = -0.1;
    
    fsm.tickFSM();

    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_BURNOUT_DETECT);
}


void coast_detect(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    ZOEM8Q0 gps;
    sensor_pointers.GPSPointer = &gps;
    gps.position_lock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->state_data.state_az = 60;
    for(int i = 0; i < 100; i++){
        fsm.tickFSM();
        chThdSleep(10);
    }
   
    sensor_pointers.sensorDataPointer->state_data.state_az = -0.1;
    
    for(int i = 0; i < 100; i++){
        sensor_pointers.sensorDataPointer->state_data.state_vz = 100 - i;
        fsm.tickFSM();
        chThdSleep(10);
    }

    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_COAST);
}


void apogee_detect(){
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    ZOEM8Q0 gps;
    sensor_pointers.GPSPointer = &gps;
    gps.position_lock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->state_data.state_az = 60;
    for(int i = 0; i < 100; i++){
        fsm.tickFSM();
        chThdSleep(10);
    }
   
    sensor_pointers.sensorDataPointer->state_data.state_az = -0.1;
    
    for(int i = 0; i < 100; i++){
        sensor_pointers.sensorDataPointer->state_data.state_vz = 100 - i;
        fsm.tickFSM();
        chThdSleep(10);
    }

    sensor_pointers.sensorDataPointer->state_data.state_vz = -1;
    fsm.tickFSM();

    TEST_ASSERT_EQUAL(sensor_pointers.sensorDataPointer->rocketState_data.rocketState, STATE_APOGEE_DETECT);
}


void run_tests() {
    UNITY_BEGIN();
    // sensor_pointers.sensorDataPointer = &sensorData;
    RUN_TEST(initial_state);
    RUN_TEST(gps_lock_detect);
    RUN_TEST(launch_detect);
    RUN_TEST(boost_detect);
    RUN_TEST(burnout_detect);
    RUN_TEST(coast_detect);
    RUN_TEST(apogee_detect);


    UNITY_END();
}

void setup() {
    delay(2000);
    chBegin(run_tests);
}

void loop() {}
