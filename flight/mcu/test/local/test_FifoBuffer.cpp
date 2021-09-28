//
// Created by 16182 on 9/27/2021.
//

#include <gtest/gtest.h>

#include "FifoBuffer.h"

TEST(FifoBufferTest, push_success_test) {
    int arr[10];
    FifoBuffer buffer(arr, 10, sizeof(int));

    for(int i = 0; i < 10; i++){
        ASSERT_TRUE(buffer.push(&i));
    }
}

TEST(FifoBufferTest, push_fail_test) {
    int arr[10];
    FifoBuffer buffer(arr, 10, sizeof(int));

    for(int i = 0; i < 10; i++){
        buffer.push(&i);
    }

    int a = 0;
    ASSERT_FALSE(buffer.push(&a));
}

