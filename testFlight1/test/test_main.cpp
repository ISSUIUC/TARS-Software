#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <unity.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.h" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "thresholds.h"
#include "pins.h"


#define SDTESTING

//test_[function]_[test being done]

//------------------------------------CONTROL-TESTS---------------------------------------
//Control test. Will always pass
void test_alwaysPasses(void){
    TEST_ASSERT_TRUE(true);
}

//------------------------------------SD-TESTING------------------------------------------
//tests file naming function with no pre-existing file
void test_dataName_noFoundFile(void){
    #ifndef SDTESTING
        TEST_IGNORE();
    #endif

    char test_data_name[16] = "test_data";
    TEST_ASSERT_EQUAL_STRING_LEN("test_data.csv", name_dataLog(test_data_name), 13);
}

//tests file naming function with 1 pre-exisiting file
void test_dataName_testcsvFound(void){
    #ifndef SDTESTING
        TEST_IGNORE();
    #endif

    File testFile = SD.open("test_data.csv", FILE_WRITE);
    testFile.write("e");
    testFile.close();

    if(!SD.exists("test_data.csv")){
        TEST_IGNORE_MESSAGE ("[TEST FAILIURE] file was not created properly by test");
    }

    char test_data_name[16] = "test_data";
    TEST_ASSERT_EQUAL_STRING_LEN("test_data1.csv", name_dataLog(test_data_name), 14);
    SD.remove("test_data.csv");
}

//----------------------------------------------------------------------------------------
void setup(){
    Serial.begin(115200);
    while (!Serial) {}

    #ifdef SDTESTING
        SD.begin(BUILTIN_SDCARD);
    #endif

    UNITY_BEGIN();

    //CONTROL TEST:
    RUN_TEST(test_alwaysPasses);

    //SD TESTING:
    RUN_TEST(test_dataName_noFoundFile);
    RUN_TEST(test_dataName_testcsvFound);


    UNITY_END();
}

void loop(){
    //Not used
}