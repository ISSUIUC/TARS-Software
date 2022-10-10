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
#include <RH_RF95.h>
#include <RHSoftwareSPI.h>

#include "KX134-1211.h"  //High-G IMU Library
#include "MS5611.h"      //Barometer library
#include "SparkFunLSM9DS1.h"                       //Low-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"  //GPS Library
#include "H3LIS331DL.h"

#define LSM9_CS_AG 2
#define LSM9_CS_M 3
#define LSM9_INT1_AG 6
#define SNS_SPI_SCK 13
#define SNS_SPI_MISO 12
#define SNS_SPI_MOSI 11
#define ZOE_M8Q_CS 6
#define SNS_UART_SDO 14
#define SNS_UART_SDI 15
#define MS5611_CS 9
#define H3LIS331DL_CS 5
#define H3LIS331DL_INT1 24
#define KX122_CS 4
#define KX122_INT1 28
#define GPS_RESET 31
#define GPS_EXTINT 32
#define LSM9_INT2_AG 33
#define LSM9_INT_M 34
#define RFM95_CS 41
#define RFM95_RST 20
#define RFM95_INT 40

#define RF95_FREQ 434.0

KX134 highGimu;
LSM9DS1 lowGimu;
SFE_UBLOX_GNSS gps;

MS5611 barometer{MS5611_CS};

RHSoftwareSPI s;

bool test_barometer(){
    barometer.init();
    barometer.read(12);

    float temp = barometer.getTemperature() *
        0.01;
    float pressure = barometer.getPressure()*0.01;
    bool did_change = false;
    for(int i = 0; i < 20; i++){
        barometer.read(12);
        float temp_new = barometer.getTemperature() *
            0.01;
        float pressure_new = barometer.getPressure()*0.01;
        if(temp_new != temp) {
            Serial.print("Temperature: "); 
            Serial.println(temp_new);
            did_change = true;
        }
        if (pressure != pressure_new) {
            Serial.print("Pressure: ");
            Serial.println(pressure_new);
        }
        
        delay(100);
    }
    

    return did_change;
}

bool test_lowg(){
    if(!lowGimu.beginSPI(LSM9_CS_AG, LSM9_CS_M)){
        return false;
    }

    for(int i = 0; i < 100; i++){
      lowGimu.readAccel();
      float accel = lowGimu.calcAccel(lowGimu.ax);
      Serial.print("Accel X: "); Serial.println(accel);
      lowGimu.readMag();
      float mag = lowGimu.calcMag(lowGimu.mx);
      Serial.print("Mag x: "); Serial.println(mag);
      delay(100);
    }
    


    return true;
}


bool test_highg(){
    LIS331 highg{};
    highg.setSPICSPin(H3LIS331DL_CS);
    highg.begin(LIS331::comm_mode::USE_SPI);
    int16_t x,y,z;
    highg.readAxes(x,y,z);
    
    for(int i = 0; i < 100; i++){
    highg.readAxes(x,y,z);
    Serial.println(x);
    Serial.println(y);
    Serial.println(z);
    delay(100);
    }
}

bool test_gps(){

    // if(!gps.begin(SPI, ZOE_M8Q_CS, 1000000)){
    //     return false;
    // }

    // //cant read data because gps needs time to connect
    s.setPins(39, 26, 27);
    RH_RF95 rf95(RFM95_CS, RFM95_INT, s);
    // return true;
    pinMode(RFM95_RST, OUTPUT);
    // pinMode(RFM95_EN, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    // digitalWrite(RFM95_EN, HIGH);

    delay(100);

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        Serial.println("Radio Initialization Failed");
        // while (1);
        return -1;
    }
    Serial.println("Radio Initialized");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        // Serial.println("setFrequency Failed");
        // while (1);
        return -1;
    }    
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
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
    pinMode(LSM9_CS_AG, OUTPUT);
    digitalWrite(LSM9_CS_AG, HIGH);
    pinMode(LSM9_CS_M, OUTPUT);
    digitalWrite(LSM9_CS_M, HIGH);
    pinMode(ZOE_M8Q_CS, OUTPUT);
    digitalWrite(ZOE_M8Q_CS, HIGH);
    pinMode(MS5611_CS, OUTPUT);
    digitalWrite(MS5611_CS, HIGH);
    pinMode(KX122_CS, OUTPUT);
    digitalWrite(KX122_CS, HIGH);
    pinMode(H3LIS331DL_CS, OUTPUT);
    digitalWrite(H3LIS331DL_CS, HIGH);
    pinMode(RFM95_CS, OUTPUT);
    digitalWrite(RFM95_CS, HIGH);
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
            case '5':
                pinMode(27, OUTPUT);
                digitalWrite(27, HIGH); 
                delay(1000);
                digitalWrite(27, LOW); 
break;
            default:
                Serial.println("unrecognized input, expected one of [1234]");
        }
    }
}
