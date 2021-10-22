#include<Arduino.h>
#include<unity.h>
#include<ch.h>
#include<ChRt.h>


#define COMPILE_TARGET

#include "FifoBuffer.h"

void int_create(){
    int a=5;
    TEST_ASSERT_EQUAL(5,a);
    TEST_ASSERT_NOT_EQUAL(1,a);
}

void test_size_0(){
    int data[10];
    int i1 = 100;
    GenericFifoBuffer buffer(data, 0, sizeof(int));    
    TEST_ASSERT_FALSE(buffer.push(&i1));
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

void untyped_fifo_buffer_push_failure() {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 10; i++) {
        (void)buffer.push(&i);
    }

    int a = 0;
    TEST_ASSERT_FALSE(buffer.push(&a));
}

void untyped_fifo_buffer_pop_one_success() {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    int i = 100;
    TEST_ASSERT_TRUE(buffer.push(&i));

    int v = 0;
    TEST_ASSERT_TRUE(buffer.pop(&v));
    TEST_ASSERT_EQUAL(v, 100);
}

void untyped_fifo_buffer_pop_many_success() {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    int i1 = 100;
    TEST_ASSERT_TRUE(buffer.push(&i1));
    int i2 = 101;
    TEST_ASSERT_TRUE(buffer.push(&i2));
    int i3 = 102;
    TEST_ASSERT_TRUE(buffer.push(&i3));

    int v = 0;
    TEST_ASSERT_TRUE(buffer.pop(&v));
    TEST_ASSERT_EQUAL(v, 100);
    TEST_ASSERT_TRUE(buffer.pop(&v));
    TEST_ASSERT_EQUAL(v, 101);
    TEST_ASSERT_TRUE(buffer.pop(&v));
    TEST_ASSERT_EQUAL(v, 102);
}

void untyped_fifo_buffer_full_buffer_recovery() {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 10; i++) {
        (void)buffer.push(&i);
    }

    int a = 0;
    TEST_ASSERT_FALSE(buffer.push(&a));

    int v = 0;
    TEST_ASSERT_TRUE(buffer.pop(&v));
    TEST_ASSERT_EQUAL(v, 0);
    TEST_ASSERT_TRUE(buffer.push(&a));
}

void fifo_buffer_push_success() {
    FifoBuffer<int, 10> buffer{};
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_TRUE(buffer.push(i));
    }
}

void fifo_buffer_push_failure() {
    FifoBuffer<int, 10> buffer{};

    for (int i = 0; i < 10; i++) {
        (void)buffer.push(i);
    }

    TEST_ASSERT_FALSE(buffer.push(0));
}

void fifo_buffer_pop_one_success() {
    FifoBuffer<int, 10> buffer{};

    TEST_ASSERT_TRUE(buffer.push(100));

    int v = 0;
    TEST_ASSERT_TRUE(buffer.pop(&v));
    TEST_ASSERT_EQUAL(v, 100);
}

void fifo_buffer_pop_many_success() {
    FifoBuffer<int, 10> buffer{};

    TEST_ASSERT_TRUE(buffer.push(100));
    TEST_ASSERT_TRUE(buffer.push(101));
    TEST_ASSERT_TRUE(buffer.push(102));

    int v = 0;

    buffer.pop(&v);
    TEST_ASSERT_EQUAL(v, 100);

    buffer.pop(&v);
    TEST_ASSERT_EQUAL(v, 101);

    buffer.pop(&v);
    TEST_ASSERT_EQUAL(v, 102);
}

void fifo_buffer_wrap_success() {
    FifoBuffer<int, 10> buffer{};

    int v = 0;

    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(buffer.push(i));

        buffer.pop(&v);
        TEST_ASSERT_EQUAL(v, i);
    }
}

void fifo_buffer_full_buffer_recovery() {
    FifoBuffer<int, 10> buffer{};

    for (int i = 0; i < 10; i++) {
        (void)buffer.push(i);
    }

    TEST_ASSERT_FALSE(buffer.push(0));

    int v = 0;

    buffer.pop(&v);
    TEST_ASSERT_EQUAL(v, 0);
    TEST_ASSERT_TRUE(buffer.push(0));
}

void run_static_test_cases() {

	// Very simple tests
    RUN_TEST(int_create);
	RUN_TEST(test_size_0);

    // Tests from test_local_FifoBuffer.cpp
    RUN_TEST(untyped_fifo_buffer_wrap_success);
    RUN_TEST(untyped_fifo_buffer_push_success);
    RUN_TEST(untyped_fifo_buffer_push_failure);
    RUN_TEST(untyped_fifo_buffer_pop_one_success);
    RUN_TEST(untyped_fifo_buffer_pop_many_success);
    RUN_TEST(untyped_fifo_buffer_full_buffer_recovery);
    RUN_TEST(fifo_buffer_push_success);
    RUN_TEST(fifo_buffer_push_failure);
    RUN_TEST(fifo_buffer_pop_one_success);
    RUN_TEST(fifo_buffer_pop_many_success);
    RUN_TEST(fifo_buffer_wrap_success);
    RUN_TEST(fifo_buffer_full_buffer_recovery);
}
