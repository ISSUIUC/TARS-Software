#include<Arduino.h>
#include<unity.h>
#include<ch.h>
#include<ChRt.h>


#define COMPILE_TARGET

#include "FifoBuffer.h"

void test(){
    int a=5;
    TEST_ASSERT_EQUAL(1,1);
    TEST_ASSERT_EQUAL(1,a);
}

void untyped_fifo_buffer_wrap_success() {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(buffer.push(&i));
        int v = 0;
        TEST_ASSERT_TRUE(buffer.pop(&v));
        TEST_ASSERT_EQUAL(v, i);
    }
}

void untyped_fifo_buffer_push_success() {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_TRUE(buffer.push(&i));
    }
}

// TEST(untyped_fifo_buffer, push_failure) {
//     int data[10];
//     GenericFifoBuffer buffer(data, 10, sizeof(int));

//     for (int i = 0; i < 10; i++) {
//         (void)buffer.push(&i);
//     }

//     int a = 0;
//     ASSERT_FALSE(buffer.push(&a));
// }

// TEST(untyped_fifo_buffer, pop_one_success) {
//     int data[10];
//     GenericFifoBuffer buffer(data, 10, sizeof(int));

//     int i = 100;
//     ASSERT_TRUE(buffer.push(&i));

//     int v = 0;
//     ASSERT_TRUE(buffer.pop(&v));
//     ASSERT_EQ(v, 100);
// }

// TEST(untyped_fifo_buffer, pop_many_success) {
//     int data[10];
//     GenericFifoBuffer buffer(data, 10, sizeof(int));

//     int i1 = 100;
//     ASSERT_TRUE(buffer.push(&i1));
//     int i2 = 101;
//     ASSERT_TRUE(buffer.push(&i2));
//     int i3 = 102;
//     ASSERT_TRUE(buffer.push(&i3));

//     int v = 0;
//     ASSERT_TRUE(buffer.pop(&v));
//     ASSERT_EQ(v, 100);
//     ASSERT_TRUE(buffer.pop(&v));
//     ASSERT_EQ(v, 101);
//     ASSERT_TRUE(buffer.pop(&v));
//     ASSERT_EQ(v, 102);
// }

// TEST(untyped_fifo_buffer, full_buffer_recovery) {
//     int data[10];
//     GenericFifoBuffer buffer(data, 10, sizeof(int));

//     for (int i = 0; i < 10; i++) {
//         (void)buffer.push(&i);
//     }

//     int a = 0;
//     ASSERT_FALSE(buffer.push(&a));

//     int v = 0;
//     ASSERT_TRUE(buffer.pop(&v));
//     ASSERT_EQ(v, 0);
//     ASSERT_TRUE(buffer.push(&a));
// }

// TEST(fifo_buffer, push_success) {
//     FifoBuffer<int, 10> buffer{};
//     for (int i = 0; i < 10; i++) {
//         ASSERT_TRUE(buffer.push(i));
//     }
// }

// TEST(fifo_buffer, push_failure) {
//     FifoBuffer<int, 10> buffer{};

//     for (int i = 0; i < 10; i++) {
//         (void)buffer.push(i);
//     }

//     ASSERT_FALSE(buffer.push(0));
// }

// TEST(fifo_buffer, pop_one_success) {
//     FifoBuffer<int, 10> buffer{};

//     ASSERT_TRUE(buffer.push(100));

//     int v = 0;
//     ASSERT_TRUE(buffer.pop(&v));
//     ASSERT_EQ(v, 100);
// }

// TEST(fifo_buffer, pop_many_success) {
//     FifoBuffer<int, 10> buffer{};

//     ASSERT_TRUE(buffer.push(100));
//     ASSERT_TRUE(buffer.push(101));
//     ASSERT_TRUE(buffer.push(102));

//     int v = 0;

//     buffer.pop(&v);
//     ASSERT_EQ(v, 100);

//     buffer.pop(&v);
//     ASSERT_EQ(v, 101);

//     buffer.pop(&v);
//     ASSERT_EQ(v, 102);
// }

// TEST(fifo_buffer, wrap_success) {
//     FifoBuffer<int, 10> buffer{};

//     int v = 0;

//     for (int i = 0; i < 100; i++) {
//         ASSERT_TRUE(buffer.push(i));

//         buffer.pop(&v);
//         ASSERT_EQ(v, i);
//     }
// }

// TEST(fifo_buffer, full_buffer_recovery) {
//     FifoBuffer<int, 10> buffer{};

//     for (int i = 0; i < 10; i++) {
//         (void)buffer.push(i);
//     }

//     ASSERT_FALSE(buffer.push(0));

//     int v = 0;

//     buffer.pop(&v);
//     ASSERT_EQ(v, 0);
//     ASSERT_TRUE(buffer.push(0));
// }
static THD_WORKING_AREA(myThreadWorkingArea, 128);

void test_size_0(){

    int data[10];
    int i1 = 100;
    GenericFifoBuffer buffer(data, 0, sizeof(int));    
    TEST_ASSERT_FALSE(buffer.push(&i1));
}

SEMAPHORE_DECL(start_thread, 0);
SEMAPHORE_DECL(thread_done, 0);
int thead_data[10];
GenericFifoBuffer thread_fifo(thead_data, 10, sizeof(int));

static THD_FUNCTION(myThread, arg) {
    chSemWait(&start_thread);

    int i = 5;
    thread_fifo.push(&i);

    chSemSignal(&thread_done);
}

void test_threads(){
    chSemSignal(&start_thread);
    chSemWait(&thread_done);


    int i;
    TEST_ASSERT(thread_fifo.pop(&i));
    TEST_ASSERT_EQUAL(i, 5);
}

void setup(){
    delay(2000);
      chSysInit();
      thread_t *tp = chThdCreateStatic(myThreadWorkingArea,
                                   sizeof(myThreadWorkingArea),
                                   NORMALPRIO,  /* Initial priority.    */
                                   myThread,    /* Thread function.     */
                                   NULL);       /* Thread parameter.    */
    UNITY_BEGIN();
    RUN_TEST(untyped_fifo_buffer_push_success);
    RUN_TEST(test_size_0);
    RUN_TEST(test_threads);
    delay(500);
    UNITY_END();

}

void loop(){

}