#include<Arduino.h>
#include<unity.h>

void test(){
    int a=5;
    TEST_ASSERT_EQUAL(1,1);
    TEST_ASSERT_EQUAL(1,a);
}

void setup(){
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test);
    delay(500);
    UNITY_END();

}

void loop(){

}