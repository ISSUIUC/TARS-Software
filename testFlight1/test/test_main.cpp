#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <unity.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"


#define SDTESTING

//test_[function]_[test being done]

//------------------------------------CHECK-TESTS-----------------------------------------
/**
 * @brief Sanity check. Will always pass
 * 
 */
void test_alwaysPasses(void){
    TEST_ASSERT_TRUE(true);
}

//------------------------------------SD-TESTING------------------------------------------
/**
 * @brief tests file naming function with no pre-existing file
 * 
 */
void test_dataName_noFoundFile(void){
    #ifndef SDTESTING
        TEST_IGNORE();
    #endif

    char test_data_name[32] = "testdatanoexist";
    char test_data_extension[6] = ".csv";
    TEST_ASSERT_EQUAL_STRING("testdatanoexist.csv", sd_file_namer(test_data_name, test_data_extension));
}

/**
 * @brief tests file naming function with 1 pre-exisiting file
 * 
 */
void test_dataName_testcsvFound(void){
    #ifndef SDTESTING
        TEST_IGNORE();
    #endif

    char preexistFilePath[32] = "testdata.csv";

    File testFile = SD.open(preexistFilePath, FILE_WRITE);
    testFile.println("deadbeef");
    testFile.flush();
    testFile.close();

    if(!SD.exists(preexistFilePath)){
        TEST_FAIL_MESSAGE ("file was not created properly by test");
    }

    char newFilename[16] = "testdata";
    char test_data_extension[6] = ".csv";
    TEST_ASSERT_EQUAL_STRING("testdata1.csv", sd_file_namer(newFilename, test_data_extension));

    SD.remove(preexistFilePath);
}


/**
 * @brief tests reformatting the data as a character array that can be written to the SD card
 * 
 */
void test_data_formatting(void){
    #ifndef SDTESTING
        TEST_IGNORE();
    #endif



    sensorDataStruct_t test_data;
    test_data.ax = 0;
    test_data.ay = 0;
    test_data.az = 0.52;
    test_data.gx = -0.432145;
    test_data.gy = 5.6;
    test_data.gz = 0.23568;
    test_data.mx = -1;
    test_data.my = 0;
    test_data.mz = 0;
    test_data.rocketState = STATE_COAST;
    test_data.timeStamp = 1302;

    test_THD_vars->current_data = test_data;
    
    

    char correctFormatting[100] = "0.0000,0.0000,0.5200,-0.4321,5.6000,0.2357,-1.0000,0.0000,0.0000,5,1302";
    TEST_ASSERT_EQUAL_STRING(correctFormatting, formatString(&test_data, LOWG_IMU));

}


//----------------------------------------------------------------------------------------
void setup(){

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_WHITE, OUTPUT);

    Serial.begin(115200);
    while (!Serial) {}

    #ifdef SDTESTING
        if(SD.begin(BUILTIN_SDCARD)){
            digitalWrite(LED_WHITE, HIGH);
        } else {
            digitalWrite(LED_RED, HIGH);
        }
    #endif

    UNITY_BEGIN();

    //CONTROL TEST:
    RUN_TEST(test_alwaysPasses);

    //SD TESTING:
    RUN_TEST(test_dataName_noFoundFile);
    RUN_TEST(test_dataName_testcsvFound);
    RUN_TEST(test_data_formatting);


    UNITY_END();
}

void loop(){
    //Not used
}
