/* main.cpp
 *   ______  ___     ___    ____
 *  /_  __/ / _ |   / _ \  / __/
 *   / /   / __ |  / , _/ _\ \
 *  /_/   /_/ |_| /_/|_| /___/
 *
 * Active Control Program
 *
 * Illinois Space Society - Avioinics Team
 *
 * Anshuk Chigullapalli
 * Josh Blustein
 * Ayberk Yaraneri
 * David Robbins
 * Matt Taylor
 * Ben Olaivar
 * Colin Kinsey
 * Grace Robbins
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "KX134-1211.h"  //High-G IMU Library
#include "MS5611.h"      //Barometer library
#include "SparkFunLSM9DS1.h"                       //Low-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"  //GPS Library

#define LSM9_CS_AG 4
#define LSM9_CS_M 5
#define LSM9_INT1_AG 6
#define SNS_SPI_SCK 13
#define SNS_SPI_MISO 12
#define SNS_SPI_MOSI 11
#define ZOE_M8Q_CS 10
#define SNS_UART_SDO 14
#define SNS_UART_SDI 15
#define MS5611_CS 21
#define H3LIS331DL_CS 23
#define H3LIS331DL_INT1 24
#define KX122_CS 27
#define KX122_INT1 28
#define GPS_RESET 31
#define GPS_EXTINT 32
#define LSM9_INT2_AG 33
#define LSM9_INT_M 34

KX134 highGimu;
LSM9DS1 lowGimu;
SFE_UBLOX_GNSS gps;

MS5611 barometer{MS5611_CS};

bool test_barometer(){
    barometer.init();
    barometer.read(12);

    float temp = barometer.getTemperature() *
        0.01;
    bool did_change = false;
    for(int i = 0; i < 20; i++){
        barometer.read(12);
        float temp_new = barometer.getTemperature() *
            0.01;
        if(temp_new != temp) did_change = true;
        Serial.print("Temperature: "); Serial.println(temp_new);
        delay(100);
    }
    

    return did_change;
}

bool test_lowg(){
    if(!lowGimu.beginSPI(LSM9_CS_AG, LSM9_CS_M)){
        return false;
    }

    lowGimu.readAccel();
    float accel = lowGimu.calcAccel(lowGimu.ax);
    Serial.print("Accel X: "); Serial.println(accel);

    return true;
}

bool test_highg(){
    highGimu.init();

    highGimu.update_data();
    float accel = highGimu.get_x_accel();

    bool did_change = false;
    for(int i = 0; i < 20; i++){
        highGimu.update_data();
        float new_accel = highGimu.get_x_accel();
        if(new_accel != accel) did_change = true;
        Serial.print("Accel X: "); Serial.println(accel);
        delay(100);
    }

    return did_change;
}

bool test_gps(){

    if(!gps.begin(SPI, ZOE_M8Q_CS, 1000000)){
        return false;
    }

    //cant read data because gps needs time to connect
    
    return true;
}

void setup() {
#if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) ||     \
    defined(BAROMETER_DEBUG) || defined(HIGHGIMU_DEBUG) || \
    defined(GPS_DEBUG) || defined(SERVO_DEBUG)
    Serial.begin(115200);
    while (!Serial) {
    }
#endif
    Serial.begin(9600);
    while(!Serial);

    SPI.begin();
    Serial.println(
    R"(
        Sensor testing program, press number to test sensor
        1: lowgimu (LSM)
        2: highgimu (KX)
        3: gps
        4: barometer
    )");
}

void loop() {
    if(Serial.available()){
        char c = Serial.read();

        switch(c){
            case '1':
                if(test_lowg()){
                    Serial.println("lowg success");
                } else {
                    Serial.println("lowg fail");
                }
                break;
            case '2':
                if(test_highg()){
                    Serial.println("highg success");
                } else {
                    Serial.println("highg fail");
                }
                break;
            case '3':
                if(test_gps()){
                    Serial.println("gps success");
                } else {
                    Serial.println("gps fail");
                }
                break;
            case '4':
                if(test_barometer()){
                    Serial.println("barometer success");
                } else {
                    Serial.println("barometer fail");
                }
                break;
            default:
                Serial.println("unrecognized input, expected one of [1234]");
        }
    }
}
