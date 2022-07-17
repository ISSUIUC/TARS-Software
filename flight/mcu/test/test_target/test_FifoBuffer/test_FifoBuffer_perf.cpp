// #include <Arduino.h>
// #include <ChRt.h>
// #include <ch.h>
// #include <unity.h>

// #define COMPILE_TARGET

// #include "FifoBuffer.h"

// // Define working area
// static THD_WORKING_AREA(producerPerfThreadWorkingArea1, 128);
// static THD_WORKING_AREA(consumerPerfThreadWorkingArea1, 128);

// /******************************************************************************/
// /* PRODUCER-CONSUMER PERF THREAD TEST */

// SEMAPHORE_DECL(start_producer_perf_thread, 0);
// SEMAPHORE_DECL(done_producer_perf_thread, 0);

// SEMAPHORE_DECL(start_comsumer_perf_thread, 0);
// SEMAPHORE_DECL(done_consumer_perf_thread, 0);

// int thread_perf_data[10];
// GenericFifoBuffer thread_perf_fifo(thread_perf_data, 10, sizeof(int));

// static THD_FUNCTION(producer_perf_thread, arg) {
//     chSemWait(&start_producer_perf_thread);

//     for (int i = 0; i < 10000; i++) {
//         while (!thread_perf_fifo.push(&i)) {
//             // let the other thread get the lock if the buffer is full
//             chThdSleep(1);
//         };
//     }

//     chSemSignal(&done_producer_perf_thread);
// }

// static THD_FUNCTION(consumer_perf_thread, arg) {
//     chSemWait(&start_comsumer_perf_thread);

//     for (int j = 0; j < 10000; j++) {
//         int i;
//         while (!thread_perf_fifo.pop(&i)) {
//             // let the other thread get the lock if the buffer is empty
//             chThdSleep(1);
//         };

//         TEST_ASSERT_EQUAL(i, j);
//     }

//     chSemSignal(&done_consumer_perf_thread);
// }

// void test_producer_performance_perf() {
//     chThdCreateStatic(producerPerfThreadWorkingArea1,
//                       sizeof(producerPerfThreadWorkingArea1),
//                       NORMALPRIO,           /* Initial priority.    */
//                       producer_perf_thread, /* Thread function.     */
//                       NULL);                /* Thread parameter.    */
//     chThdCreateStatic(consumerPerfThreadWorkingArea1,
//                       sizeof(consumerPerfThreadWorkingArea1),
//                       NORMALPRIO,           /* Initial priority.    */
//                       consumer_perf_thread, /* Thread function.     */
//                       NULL);                /* Thread parameter.    */

//     // let the two threads start so thread startup time does not impact perf
//     // testing
//     chThdSleep(100);

//     chSemSignal(&start_producer_perf_thread);
//     chSemSignal(&start_comsumer_perf_thread);
//     auto begin = chVTGetSystemTime();

//     chSemWait(&done_producer_perf_thread);
//     chSemWait(&done_consumer_perf_thread);
//     // measure the time the test took to run
//     auto end = chVTGetSystemTime();
//     auto tick_time = 1.f / CH_CFG_ST_FREQUENCY;
//     auto elapsed_time_ms = (end - begin) * tick_time * 1000;
//     Serial.printf("PERF: 10000 pushes in %f ms\n", elapsed_time_ms);

//     // test that the throughput was higher than 10000 ints in 100 ms
//     TEST_ASSERT_LESS_THAN(100, elapsed_time_ms);
// }

// /******************************************************************************/
// /* TYPED PRODUCER-CONSUMER PERF THREAD TEST */

// FifoBuffer<int, 10> typed_thread_perf_fifo{};

// static THD_FUNCTION(typed_producer_perf_thread, arg) {
//     chSemWait(&start_producer_perf_thread);

//     for (int i = 0; i <= 10000; i++) {
//         while (!typed_thread_perf_fifo.push(i)) {
//             // let the other thread get the lock if the buffer is full
//             chThdSleep(1);
//         };
//     }

//     chSemSignal(&done_producer_perf_thread);
// }

// static THD_FUNCTION(typed_consumer_perf_thread, arg) {
//     chSemWait(&start_comsumer_perf_thread);

//     for (int j = 0; j <= 10000; j++) {
//         int i;
//         while (!typed_thread_perf_fifo.pop(&i)) {
//             // let the other thread get the lock if the buffer is empty
//             chThdSleep(1);
//         };

//         TEST_ASSERT_EQUAL(i, j);
//     }

//     chSemSignal(&done_consumer_perf_thread);
// }

// void typed_test_producer_performance_perf() {
//     chThdCreateStatic(producerPerfThreadWorkingArea1,
//                       sizeof(producerPerfThreadWorkingArea1),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_producer_perf_thread, /* Thread function.     */
//                       NULL);                      /* Thread parameter.    */
//     chThdCreateStatic(consumerPerfThreadWorkingArea1,
//                       sizeof(consumerPerfThreadWorkingArea1),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_consumer_perf_thread, /* Thread function.     */
//                       NULL);                      /* Thread parameter.    */

//     // let the two threads start so thread startup time does not impact perf
//     // testing
//     chThdSleep(100);

//     chSemSignal(&start_producer_perf_thread);
//     chSemSignal(&start_comsumer_perf_thread);
//     auto begin = chVTGetSystemTime();

//     chSemWait(&done_producer_perf_thread);
//     chSemWait(&done_consumer_perf_thread);

//     // measure the time the test took to run
//     auto end = chVTGetSystemTime();
//     auto tick_time = 1.f / CH_CFG_ST_FREQUENCY;
//     auto elapsed_time_ms = (end - begin) * tick_time * 1000;
//     Serial.printf("PERF: 10000 pushes in %f ms\n", elapsed_time_ms);

//     // test that the throughput was higher than 10000 ints in 100 ms
//     TEST_ASSERT_LESS_THAN(100, elapsed_time_ms);
// }

// /******************************************************************************/
// /* MANY PRODUCER ONE CONSUMER PERF THREAD TEST */

// static THD_WORKING_AREA(main_consumer_WA, 128);
// static THD_WORKING_AREA(producer1_WA, 128);
// static THD_WORKING_AREA(producer2_WA, 128);
// static THD_WORKING_AREA(producer3_WA, 128);
// static THD_WORKING_AREA(producer4_WA, 128);
// static THD_WORKING_AREA(producer5_WA, 128);

// int data_many_0[10];
// int data_many_1[10];
// int data_many_2[10];
// int data_many_3[10];
// int data_many_4[10];

// GenericFifoBuffer prod_1_fifo(data_many_0, 10, sizeof(int));
// GenericFifoBuffer prod_2_fifo(data_many_1, 10, sizeof(int));
// GenericFifoBuffer prod_3_fifo(data_many_2, 10, sizeof(int));
// GenericFifoBuffer prod_4_fifo(data_many_3, 10, sizeof(int));
// GenericFifoBuffer prod_5_fifo(data_many_4, 10, sizeof(int));

// SEMAPHORE_DECL(main_consumer_start, 0);
// SEMAPHORE_DECL(main_consumer_end, 0);

// SEMAPHORE_DECL(producer_start, 0);
// SEMAPHORE_DECL(producer_end, 0);

// static THD_FUNCTION(many_producer_thread, arg) {
//     GenericFifoBuffer* buffer = (GenericFifoBuffer*)arg;
//     chSemWait(&producer_start);

//     for (int i = 0; i < 10000; i++) {
//         while (!buffer->push(&i)) {
//             // let the other thread get the lock if the buffer is full
//             chThdSleep(1);
//         };
//     }

//     chSemSignal(&producer_end);
// }

// static THD_FUNCTION(main_consumer_thread, arg) {
//     chSemWait(&main_consumer_start);

//     for (int i = 0; i < 10000; i++) {
//         int v1, v2, v3, v4, v5;
//         while (!prod_1_fifo.pop(&v1)) {
//             chThdSleep(1);
//         };
//         while (!prod_2_fifo.pop(&v2)) {
//             chThdSleep(1);
//         };
//         while (!prod_3_fifo.pop(&v3)) {
//             chThdSleep(1);
//         };
//         while (!prod_4_fifo.pop(&v4)) {
//             chThdSleep(1);
//         };
//         while (!prod_5_fifo.pop(&v5)) {
//             chThdSleep(1);
//         };

//         TEST_ASSERT_EQUAL(i, v1);
//         TEST_ASSERT_EQUAL(i, v2);
//         TEST_ASSERT_EQUAL(i, v3);
//         TEST_ASSERT_EQUAL(i, v4);
//         TEST_ASSERT_EQUAL(i, v5);
//     }

//     chSemSignal(&main_consumer_end);
// }

// void test_many_producer_perf() {
//     chThdCreateStatic(producer1_WA, sizeof(producer1_WA),
//                       NORMALPRIO,           /* Initial priority.    */
//                       many_producer_thread, /* Thread function.     */
//                       &prod_1_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer2_WA, sizeof(producer2_WA),
//                       NORMALPRIO,           /* Initial priority.    */
//                       many_producer_thread, /* Thread function.     */
//                       &prod_2_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer3_WA, sizeof(producer3_WA),
//                       NORMALPRIO,           /* Initial priority.    */
//                       many_producer_thread, /* Thread function.     */
//                       &prod_3_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer4_WA, sizeof(producer4_WA),
//                       NORMALPRIO,           /* Initial priority.    */
//                       many_producer_thread, /* Thread function.     */
//                       &prod_4_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer5_WA, sizeof(producer5_WA),
//                       NORMALPRIO,           /* Initial priority.    */
//                       many_producer_thread, /* Thread function.     */
//                       &prod_5_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(main_consumer_WA, sizeof(main_consumer_WA),
//                       NORMALPRIO,           /* Initial priority.    */
//                       main_consumer_thread, /* Thread function.     */
//                       NULL);                /* Thread parameter.    */

//     // wait for threads to start
//     chThdSleep(100);

//     chSemSignal(&main_consumer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     auto begin = chVTGetSystemTime();

//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&main_consumer_end);

//     // measure the time the test took to run
//     auto end = chVTGetSystemTime();
//     auto tick_time = 1.f / CH_CFG_ST_FREQUENCY;
//     auto elapsed_time_ms = (end - begin) * tick_time * 1000;
//     Serial.printf("PERF: 10000 pushes in %f ms\n", elapsed_time_ms);

//     // test that the throughput was higher than 50000 ints over 5 buffers in
//     100
//     // ms
//     TEST_ASSERT_LESS_THAN(100, elapsed_time_ms);
// }

// /******************************************************************************/
// /* TYPED MANY PRODUCER ONE CONSUMER PERF THREAD TEST */

// FifoBuffer<int, 10> typed_prod_1_fifo{};
// FifoBuffer<int, 10> typed_prod_2_fifo{};
// FifoBuffer<int, 10> typed_prod_3_fifo{};
// FifoBuffer<int, 10> typed_prod_4_fifo{};
// FifoBuffer<int, 10> typed_prod_5_fifo{};

// static THD_FUNCTION(typed_many_producer_thread, arg) {
//     auto buffer = (FifoBuffer<int, 10>*)arg;
//     chSemWait(&producer_start);

//     for (int i = 0; i < 10000; i++) {
//         while (!buffer->push(i)) {
//             // let the other thread get the lock if the buffer is full
//             chThdSleep(1);
//         };
//     }

//     chSemSignal(&producer_end);
// }

// static THD_FUNCTION(typed_main_consumer_thread, arg) {
//     chSemWait(&main_consumer_start);

//     for (int i = 0; i < 10000; i++) {
//         int v1, v2, v3, v4, v5;
//         while (!typed_prod_1_fifo.pop(&v1)) {
//             chThdSleep(1);
//         };
//         while (!typed_prod_2_fifo.pop(&v2)) {
//             chThdSleep(1);
//         };
//         while (!typed_prod_3_fifo.pop(&v3)) {
//             chThdSleep(1);
//         };
//         while (!typed_prod_4_fifo.pop(&v4)) {
//             chThdSleep(1);
//         };
//         while (!typed_prod_5_fifo.pop(&v5)) {
//             chThdSleep(1);
//         };

//         TEST_ASSERT_EQUAL(i, v1);
//         TEST_ASSERT_EQUAL(i, v2);
//         TEST_ASSERT_EQUAL(i, v3);
//         TEST_ASSERT_EQUAL(i, v4);
//         TEST_ASSERT_EQUAL(i, v5);
//     }

//     chSemSignal(&main_consumer_end);
// }

// void typed_test_many_producer_perf() {
//     chThdCreateStatic(producer1_WA, sizeof(producer1_WA),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_many_producer_thread, /* Thread function.     */
//                       &typed_prod_1_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer2_WA, sizeof(producer2_WA),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_many_producer_thread, /* Thread function.     */
//                       &typed_prod_2_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer3_WA, sizeof(producer3_WA),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_many_producer_thread, /* Thread function.     */
//                       &typed_prod_3_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer4_WA, sizeof(producer4_WA),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_many_producer_thread, /* Thread function.     */
//                       &typed_prod_4_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(producer5_WA, sizeof(producer5_WA),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_many_producer_thread, /* Thread function.     */
//                       &typed_prod_5_fifo);        /* Thread parameter.    */
//     chThdCreateStatic(main_consumer_WA, sizeof(main_consumer_WA),
//                       NORMALPRIO,                 /* Initial priority.    */
//                       typed_main_consumer_thread, /* Thread function.     */
//                       NULL);                      /* Thread parameter.    */

//     // wait for threads to start
//     chThdSleep(100);

//     chSemSignal(&main_consumer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     chSemSignal(&producer_start);
//     auto begin = chVTGetSystemTime();

//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&producer_end);
//     chSemWait(&main_consumer_end);
//     // measure the time the test took to run
//     auto end = chVTGetSystemTime();
//     auto tick_time = 1.f / CH_CFG_ST_FREQUENCY;
//     auto elapsed_time_ms = (end - begin) * tick_time * 1000;
//     Serial.printf("PERF: 10000 pushes in %f ms\n", elapsed_time_ms);

//     // test that the throughput was higher than 50000 ints over 5 buffers in
//     100
//     // ms
//     TEST_ASSERT_LESS_THAN(100, elapsed_time_ms);
// }

// /******************************************************************************/
// /* RUN THREADED TEST CASES */

// void run_performance_test_cases() {
//     RUN_TEST(test_producer_performance_perf);
//     RUN_TEST(typed_test_producer_performance_perf);
//     RUN_TEST(test_many_producer_perf);
//     RUN_TEST(typed_test_many_producer_perf);
// }
