
// #include <Arduino.h>
// // #include <Threads.h>
// #include <unity.h>

// #define COMPILE_TARGET

// #include "FifoBuffer.h"

// void test_push_success(void) {
//     int data[10];
//     GenericFifoBuffer buffer(data, 10, sizeof(int));

//     for (int i = 0; i < 10; i++) {
//         TEST_ASSERT_TRUE(buffer.push(&i));
//     }
// }

// void test_trivial(void) {
//     TEST_ASSERT_TRUE(1 == 1);
// }

// void setup() {
//     delay(2000);
//     UNITY_BEGIN();
//     RUN_TEST(test_trivial);
//     delay(500);
//     UNITY_END();
// }

// void loop() {}
