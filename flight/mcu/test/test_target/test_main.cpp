#include <Arduino.h>
#include <ChRt.h>
#include <unity.h>

void run_fsm_tests();
void run_fifo_tests();

void run_tests() {
    UNITY_BEGIN();

    run_fsm_tests();
    run_fifo_tests();

    UNITY_END();
}

void setup() {
    while (!Serial)
        ;
    chBegin(run_tests);
}

void loop() {}