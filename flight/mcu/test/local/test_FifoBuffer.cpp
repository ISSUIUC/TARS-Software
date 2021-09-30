#include <gtest/gtest.h>

#include "FifoBuffer.h"

TEST(untyped_fifo_buffer, push_success) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 10; i++) {
        ASSERT_TRUE(buffer.push(&i));
    }
}

TEST(untyped_fifo_buffer, push_failure) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 10; i++) {
        (void)buffer.push(&i);
    }

    int a = 0;
    ASSERT_FALSE(buffer.push(&a));
}

TEST(untyped_fifo_buffer, pop_one_success) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    int i = 100;
    ASSERT_TRUE(buffer.push(&i));

    int v = 0;
    ASSERT_TRUE(buffer.pop(&v));
    ASSERT_EQ(v, 100);
}

TEST(untyped_fifo_buffer, pop_many_success) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    int i1 = 100;
    ASSERT_TRUE(buffer.push(&i1));
    int i2 = 101;
    ASSERT_TRUE(buffer.push(&i2));
    int i3 = 102;
    ASSERT_TRUE(buffer.push(&i3));

    int v = 0;
    ASSERT_TRUE(buffer.pop(&v));
    ASSERT_EQ(v, 100);
    ASSERT_TRUE(buffer.pop(&v));
    ASSERT_EQ(v, 101);
    ASSERT_TRUE(buffer.pop(&v));
    ASSERT_EQ(v, 102);
}

TEST(untyped_fifo_buffer, wrap_success) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 100; i++) {
        ASSERT_TRUE(buffer.push(&i));
        int v = 0;
        ASSERT_TRUE(buffer.pop(&v));
        ASSERT_EQ(v, i);
    }
}

TEST(untyped_fifo_buffer, full_buffer_recovery) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for (int i = 0; i < 10; i++) {
        (void)buffer.push(&i);
    }

    int a = 0;
    ASSERT_FALSE(buffer.push(&a));

    int v = 0;
    ASSERT_TRUE(buffer.pop(&v));
    ASSERT_EQ(v, 0);
    ASSERT_TRUE(buffer.push(&a));
}
