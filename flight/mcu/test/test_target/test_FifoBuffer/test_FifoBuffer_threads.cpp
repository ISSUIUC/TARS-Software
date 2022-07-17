// #include <Arduino.h>
// #include <ChRt.h>
// #include <ch.h>
// #include <unity.h>

// #define COMPILE_TARGET

// #include "FifoBuffer.h"

// // Define working area
// static THD_WORKING_AREA(myThreadWorkingArea, 128);

// static THD_WORKING_AREA(producerThreadWorkingArea1, 128);
// static THD_WORKING_AREA(consumerThreadWorkingArea1, 128);

// static THD_WORKING_AREA(popThreadWorkingArea1, 128);
// static THD_WORKING_AREA(pushThreadWorkingArea1, 128);

// static THD_WORKING_AREA(pushThreadWorkingArea2, 128);
// static THD_WORKING_AREA(pushThreadWorkingArea3, 128);

// static THD_WORKING_AREA(typed_myThreadWorkingArea, 128);

// static THD_WORKING_AREA(typed_producerThreadWorkingArea1, 128);
// static THD_WORKING_AREA(typed_consumerThreadWorkingArea1, 128);

// static THD_WORKING_AREA(typed_popThreadWorkingArea1, 128);
// static THD_WORKING_AREA(typed_pushThreadWorkingArea1, 128);

// static THD_WORKING_AREA(typed_pushThreadWorkingArea2, 128);
// static THD_WORKING_AREA(typed_pushThreadWorkingArea3, 128);

// /******************************************************************************/
// /* SIMPLE PUSHING IN THREAD TEST TYPED */

// SEMAPHORE_DECL(typed_start_thread, 0);
// SEMAPHORE_DECL(typed_thread_done, 0);
// FifoBuffer<int, 10> typed_thread_fifo{};

// static THD_FUNCTION(typed_myThread, arg) {
//     chSemWait(&typed_start_thread);

//     int i = 5;
//     typed_thread_fifo.push(i);

//     chSemSignal(&typed_thread_done);
// }

// void typed_test_thread_push() {
//     chSemSignal(&typed_start_thread);
//     chSemWait(&typed_thread_done);

//     int i;
//     TEST_ASSERT(typed_thread_fifo.pop(&i));
//     TEST_ASSERT_EQUAL(i, 5);
// }

// /******************************************************************************/
// /* PRODUCER-CONSUMER THREAD TEST TYPED */

// SEMAPHORE_DECL(typed_start_producerThread1, 0);
// SEMAPHORE_DECL(typed_done_producerThread1, 0);

// SEMAPHORE_DECL(typed_start_consumerThread1, 0);
// SEMAPHORE_DECL(typed_done_consumerThread1, 0);

// FifoBuffer<int, 10> typed_thread_fifo2{};

// static THD_FUNCTION(typed_producerThread1, arg) {
//     chSemWait(&typed_start_producerThread1);

//     int i = 0;

//     for (i = 0; i <= 100; i++) {
//         while (!typed_thread_fifo2.push(i)) {
//             // let the other thread get the lock if the buffer is full
//             chThdSleep(1);
//         }
//     }

//     chSemSignal(&typed_done_producerThread1);
// }

// static THD_FUNCTION(typed_consumerThread1, arg) {
//     chSemWait(&typed_start_consumerThread1);

//     int i, j;

//     for (j = 0; j <= 100; j++) {
//         while (!typed_thread_fifo2.pop(&i)) {
//             // let the other thread get the lock if the buffer is empty
//             chThdSleep(1);
//         }

//         TEST_ASSERT_EQUAL(i, j);
//     }

//     chSemSignal(&typed_done_consumerThread1);
// }

// void typed_test_producer_consumer() {
//     chSemSignal(&typed_start_consumerThread1);
//     chSemSignal(&typed_start_producerThread1);

//     chSemWait(&typed_done_producerThread1);
//     chSemWait(&typed_done_consumerThread1);
// }

// /******************************************************************************/
// /* THREAD POP FAILING TEST TYPED */

// SEMAPHORE_DECL(typed_start_popThread1, 0);
// SEMAPHORE_DECL(typed_done_popThread1, 0);

// SEMAPHORE_DECL(typed_start_pushThread1, 0);
// SEMAPHORE_DECL(typed_done_pushThread1, 0);

// FifoBuffer<int, 10> typed_thread_fifo3{};

// static THD_FUNCTION(typed_pushThread1, arg) {
//     chSemWait(&typed_start_pushThread1);

//     int i = 0;

//     while (!typed_thread_fifo3.push(i))
//         ;

//     chSemSignal(&typed_done_pushThread1);
// }

// static THD_FUNCTION(typed_popThread1, arg) {
//     chSemWait(&typed_start_popThread1);

//     int i;

//     // Push the first time and it should work.
//     TEST_ASSERT_TRUE(typed_thread_fifo3.pop(&i));

//     TEST_ASSERT_EQUAL(i, 0);

//     // Push the second time and it should fail
//     TEST_ASSERT_FALSE(typed_thread_fifo3.pop(&i))

//     chSemSignal(&typed_done_popThread1);
// }

// void typed_test_thread_pop_fail() {
//     chSemSignal(&typed_start_pushThread1);
//     chSemWait(&typed_done_pushThread1);

//     chSemSignal(&typed_start_popThread1);
//     chSemWait(&typed_done_popThread1);
// }

// /******************************************************************************/
// /* THREAD PUSH FAILING TEST TYPED */

// SEMAPHORE_DECL(typed_start_pushThread2, 0);
// SEMAPHORE_DECL(typed_done_pushThread2, 0);

// SEMAPHORE_DECL(typed_start_pushThread3, 0);
// SEMAPHORE_DECL(typed_done_pushThread3, 0);

// FifoBuffer<int, 1> typed_thread_fifo4{};

// static THD_FUNCTION(typed_pushThread2, arg) {
//     chSemWait(&typed_start_pushThread2);

//     int i = 1;
//     TEST_ASSERT_TRUE(typed_thread_fifo4.push(i));

//     chSemSignal(&typed_done_pushThread2);
// }

// static THD_FUNCTION(typed_pushThread3, arg) {
//     chSemWait(&typed_start_pushThread3);

//     int i = 2;
//     // Fail because array is only length 1.
//     TEST_ASSERT_FALSE(typed_thread_fifo4.push(i));

//     // After failing it should still returning the pushed number
//     TEST_ASSERT_EQUAL(1, typed_thread_fifo4.pop(&i));

//     chSemSignal(&typed_done_pushThread3);
// }

// void typed_test_thread_push_fail() {
//     chSemSignal(&typed_start_pushThread2);
//     chSemWait(&typed_done_pushThread2);

//     chSemSignal(&typed_start_pushThread3);
//     chSemWait(&typed_done_pushThread3);
// }

// /******************************************************************************/
// /* SIMPLE PUSHING IN THREAD TEST */

// SEMAPHORE_DECL(start_thread, 0);
// SEMAPHORE_DECL(thread_done, 0);
// int thread_data[10];
// GenericFifoBuffer thread_fifo(thread_data, 10, sizeof(int));

// static THD_FUNCTION(myThread, arg) {
//     chSemWait(&start_thread);

//     int i = 5;
//     thread_fifo.push(&i);

//     chSemSignal(&thread_done);
// }

// void test_thread_push() {
//     chSemSignal(&start_thread);
//     chSemWait(&thread_done);

//     int i;
//     TEST_ASSERT(thread_fifo.pop(&i));
//     TEST_ASSERT_EQUAL(i, 5);
// }

// /******************************************************************************/
// /* PRODUCER-CONSUMER THREAD TEST */

// SEMAPHORE_DECL(start_producerThread1, 0);
// SEMAPHORE_DECL(done_producerThread1, 0);

// SEMAPHORE_DECL(start_consumerThread1, 0);
// SEMAPHORE_DECL(done_consumerThread1, 0);

// int thread_data2[10];
// GenericFifoBuffer thread_fifo2(thread_data2, 10, sizeof(int));

// static THD_FUNCTION(producerThread1, arg) {
//     chSemWait(&start_producerThread1);

//     for (int i = 0; i < 100; i++) {
//         while (!thread_fifo2.push(&i)) {
//             // let the other thread get the lock if the buffer is full
//             chThdSleep(1);
//         }
//     }

//     chSemSignal(&done_producerThread1);
// }

// static THD_FUNCTION(consumerThread1, arg) {
//     chSemWait(&start_consumerThread1);

//     for (int j = 0; j < 100; j++) {
//         int i;
//         while (!thread_fifo2.pop(&i)) {
//             // let the other thread get the lock if the buffer is empty
//             chThdSleep(1);
//         }

//         TEST_ASSERT_EQUAL(i, j);
//     }

//     chSemSignal(&done_consumerThread1);
// }

// void test_producer_consumer() {
//     chSemSignal(&start_consumerThread1);
//     chSemSignal(&start_producerThread1);

//     chSemWait(&done_producerThread1);
//     chSemWait(&done_consumerThread1);
// }

// /******************************************************************************/
// /* THREAD POP FAILING TEST */

// SEMAPHORE_DECL(start_popThread1, 0);
// SEMAPHORE_DECL(done_popThread1, 0);

// SEMAPHORE_DECL(start_pushThread1, 0);
// SEMAPHORE_DECL(done_pushThread1, 0);

// int thread_data3[10];
// GenericFifoBuffer thread_fifo3(thread_data3, 10, sizeof(int));

// static THD_FUNCTION(pushThread1, arg) {
//     chSemWait(&start_pushThread1);

//     int i = 0;

//     while (!thread_fifo3.push(&i))
//         ;

//     chSemSignal(&done_pushThread1);
// }

// static THD_FUNCTION(popThread1, arg) {
//     chSemWait(&start_popThread1);

//     int i;

//     // Pop the first time and it should work.
//     TEST_ASSERT_TRUE(thread_fifo3.pop(&i));

//     TEST_ASSERT_EQUAL(i, 0);

//     // Pop the second time and it should fail
//     TEST_ASSERT_FALSE(thread_fifo3.pop(&i))

//     chSemSignal(&done_popThread1);
// }

// void test_thread_pop_fail() {
//     chSemSignal(&start_pushThread1);
//     chSemWait(&done_pushThread1);

//     chSemSignal(&start_popThread1);
//     chSemWait(&done_popThread1);
// }

// /******************************************************************************/
// /* THREAD PUSH FAILING TEST */

// SEMAPHORE_DECL(start_pushThread2, 0);
// SEMAPHORE_DECL(done_pushThread2, 0);

// SEMAPHORE_DECL(start_pushThread3, 0);
// SEMAPHORE_DECL(done_pushThread3, 0);

// int thread_data4[1];
// GenericFifoBuffer thread_fifo4(thread_data4, 1, sizeof(int));

// static THD_FUNCTION(pushThread2, arg) {
//     chSemWait(&start_pushThread2);

//     int i = 1;
//     TEST_ASSERT_TRUE(thread_fifo4.push(&i));

//     chSemSignal(&done_pushThread2);
// }

// static THD_FUNCTION(pushThread3, arg) {
//     chSemWait(&start_pushThread3);

//     int i = 2;
//     // Fail because array is only length 1.
//     TEST_ASSERT_FALSE(thread_fifo4.push(&i));

//     // After failing it should still returning the pushed number
//     TEST_ASSERT_EQUAL(1, thread_fifo4.pop(&i));

//     chSemSignal(&done_pushThread3);
// }

// void test_thread_push_fail() {
//     chSemSignal(&start_pushThread2);
//     chSemWait(&done_pushThread2);

//     chSemSignal(&start_pushThread3);
//     chSemWait(&done_pushThread3);
// }

// /******************************************************************************/
// /* RUN THREADED TEST CASES */

// void run_threaded_test_cases() {
//     chThdCreateStatic(myThreadWorkingArea, sizeof(myThreadWorkingArea),
//                       NORMALPRIO, /* Initial priority.    */
//                       myThread,   /* Thread function.     */
//                       NULL);      /* Thread parameter.    */

//     chThdCreateStatic(producerThreadWorkingArea1,
//                       sizeof(producerThreadWorkingArea1),
//                       NORMALPRIO,      /* Initial priority.    */
//                       producerThread1, /* Thread function.     */
//                       NULL);           /* Thread parameter.    */
//     chThdCreateStatic(consumerThreadWorkingArea1,
//                       sizeof(consumerThreadWorkingArea1),
//                       NORMALPRIO,      /* Initial priority.    */
//                       consumerThread1, /* Thread function.     */
//                       NULL);           /* Thread parameter.    */

//     chThdCreateStatic(popThreadWorkingArea1, sizeof(popThreadWorkingArea1),
//                       NORMALPRIO, /* Initial priority.    */
//                       popThread1, /* Thread function.     */
//                       NULL);      /* Thread parameter.    */
//     chThdCreateStatic(pushThreadWorkingArea1, sizeof(pushThreadWorkingArea1),
//                       NORMALPRIO,  /* Initial priority.    */
//                       pushThread1, /* Thread function.     */
//                       NULL);       /* Thread parameter.    */

//     chThdCreateStatic(pushThreadWorkingArea2, sizeof(pushThreadWorkingArea2),
//                       NORMALPRIO,  /* Initial priority.    */
//                       pushThread2, /* Thread function.     */
//                       NULL);       /* Thread parameter.    */
//     chThdCreateStatic(pushThreadWorkingArea3, sizeof(pushThreadWorkingArea3),
//                       NORMALPRIO,  /* Initial priority.    */
//                       pushThread3, /* Thread function.     */
//                       NULL);       /* Thread parameter.    */

//     // New tests, typed
//     RUN_TEST(test_thread_push);
//     RUN_TEST(test_producer_consumer);
//     RUN_TEST(test_thread_pop_fail);
//     RUN_TEST(test_thread_push_fail);

//     chThdCreateStatic(typed_myThreadWorkingArea,
//                       sizeof(typed_myThreadWorkingArea),
//                       NORMALPRIO,     /* Initial priority.    */
//                       typed_myThread, /* Thread function.     */
//                       NULL);          /* Thread parameter.    */

//     chThdCreateStatic(typed_producerThreadWorkingArea1,
//                       sizeof(typed_producerThreadWorkingArea1),
//                       NORMALPRIO,            /* Initial priority.    */
//                       typed_producerThread1, /* Thread function.     */
//                       NULL);                 /* Thread parameter.    */
//     chThdCreateStatic(typed_consumerThreadWorkingArea1,
//                       sizeof(typed_consumerThreadWorkingArea1),
//                       NORMALPRIO,            /* Initial priority.    */
//                       typed_consumerThread1, /* Thread function.     */
//                       NULL);                 /* Thread parameter.    */

//     chThdCreateStatic(typed_popThreadWorkingArea1,
//                       sizeof(typed_popThreadWorkingArea1),
//                       NORMALPRIO,       /* Initial priority.    */
//                       typed_popThread1, /* Thread function.     */
//                       NULL);            /* Thread parameter.    */
//     chThdCreateStatic(typed_pushThreadWorkingArea1,
//                       sizeof(typed_pushThreadWorkingArea1),
//                       NORMALPRIO,        /* Initial priority.    */
//                       typed_pushThread1, /* Thread function.     */
//                       NULL);             /* Thread parameter.    */

//     chThdCreateStatic(typed_pushThreadWorkingArea2,
//                       sizeof(typed_pushThreadWorkingArea2),
//                       NORMALPRIO,        /* Initial priority.    */
//                       typed_pushThread2, /* Thread function.     */
//                       NULL);             /* Thread parameter.    */
//     chThdCreateStatic(typed_pushThreadWorkingArea3,
//                       sizeof(typed_pushThreadWorkingArea3),
//                       NORMALPRIO,        /* Initial priority.    */
//                       typed_pushThread3, /* Thread function.     */
//                       NULL);             /* Thread parameter.    */

//     // New tests
//     RUN_TEST(typed_test_thread_push);
//     RUN_TEST(typed_test_producer_consumer);
//     RUN_TEST(typed_test_thread_pop_fail);
//     RUN_TEST(typed_test_thread_push_fail);
// }
