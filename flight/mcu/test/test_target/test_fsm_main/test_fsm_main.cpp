#include <Arduino.h>
#include <ChRt.h>
#include <ch.h>
#include <unity.h>

#include "rocketFSM.h"

void initial_state() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;

    rocketFSM fsm{&sensor_pointers};
    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_INIT);
    fsm.tickFSM();
}

void gps_lock_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_IDLE);
}

void launch_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->lowG_data.ay = 6;
    fsm.tickFSM();
    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_LAUNCH_DETECT);
}

void boost_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;


    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->lowG_data.ay = 6;
    for (int i = 0; i < 100; i++) {
        fsm.tickFSM();
        chThdSleep(10);
    }

    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_BOOST);
}

void bad_data_boost_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;


    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->lowG_data.ay = 60;

    fsm.tickFSM();
    chThdSleep(10);
    sensor_pointers.sensorDataPointer->lowG_data.ay = 0;
    fsm.tickFSM();

    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_IDLE);
}

void burnout_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;


    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->lowG_data.ay = 60;
    for (int i = 0; i < 100; i++) {
        fsm.tickFSM();
        chThdSleep(10);
    }

    sensor_pointers.sensorDataPointer->lowG_data.ay = -0.1;

    fsm.tickFSM();

    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_BURNOUT_DETECT);
}

void coast_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->lowG_data.ay = 60;
    for (int i = 0; i < 100; i++) {
        fsm.tickFSM();
        chThdSleep(10);
    }

    sensor_pointers.sensorDataPointer->lowG_data.ay = -0.1;

    for (int i = 0; i < 100; i++) {
        fsm.tickFSM();
        chThdSleep(10);
    }

    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_COAST);
}

void apogee_detect() {
    sensorDataStruct_t sensorData{};
    pointers sensor_pointers{};
    sensor_pointers.sensorDataPointer = &sensorData;
    sensorData.gps_data.posLock = true;

    rocketFSM fsm{&sensor_pointers};
    fsm.tickFSM();
    sensor_pointers.sensorDataPointer->lowG_data.ay = 60;
    for (int i = 0; i < 100; i++) {
        fsm.tickFSM();
        chThdSleep(10);
    }

    sensor_pointers.sensorDataPointer->lowG_data.ay = -0.1;

    for (int i = 0; i < 100; i++) {
        fsm.tickFSM();
        chThdSleep(10);
    }

    fsm.tickFSM();

    TEST_ASSERT_EQUAL(
        sensor_pointers.sensorDataPointer->rocketState_data.rocketState,
        STATE_APOGEE_DETECT);
}

void run_fsm_tests() {
    RUN_TEST(initial_state);
    RUN_TEST(gps_lock_detect);
    RUN_TEST(launch_detect);
    RUN_TEST(boost_detect);
    RUN_TEST(burnout_detect);
    RUN_TEST(coast_detect);
    RUN_TEST(apogee_detect);
    RUN_TEST(bad_data_boost_detect);
}