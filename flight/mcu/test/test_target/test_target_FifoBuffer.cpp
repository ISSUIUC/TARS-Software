
#include <Arduino.h>
#include <unity.h>

#define COMPILE_TARGET

#include "FifoBuffer.h"


void test_push_success(void) {
    int data[10];
    GenericFifoBuffer buffer(data, 10, sizeof(int));

    for(int i = 0; i < 10; i++){
        TEST_ASSERT_TRUE(buffer.push(&i));
    }
}


void setup() {

}

void loop() {

}
