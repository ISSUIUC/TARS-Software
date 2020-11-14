#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SPI.h>

#define SERVO1_PIN 3
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 9
const unsigned int TERMINATE = 99000000;

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;


int currentAngle; //Current angle of the servos. 
int initialAngle = 0; //Initial angle of servos. This may not be zero.
float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float roll_rate; //angular velocity in the z direction
float g = 9.81; //Acceleration due to gravity in m/s
float lattitude; //current lattitude from gps
float longitude; //current longitude from gps



//Gives thread 32 bytes. STATIC, so size will not change.
static THD_WORKING_AREA(waThread, 32);
//May have to change the size
static THD_WORKING_AREA(waThread2, 32);
//Workign area thread for logger
static THD_WORKING_AREA(waThread3, 32);

//Thread to get data from sensors
static THD_FUNCTION(dataThread, arg) {
    (void)arg;
    // start the SPI library:
    SPI.begin();
    /*
    Need IMU, Pitot, and GPS data from Beaglebone...
    
    Datasheet for IMU: https://www.mouser.com/datasheet/2/389/lsm9ds1-1849526.pdf
    Datasheet for Pressure Sensor: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5611-01BA03&DocType=Data+Sheet&DocLang=English
    Datasheet for Accelerometer: https://www.mouser.com/datasheet/2/348/KX134-1211-Specifications-Rev-1.0-1659717.pdf
    Datasheet for GPS: https://www.u-blox.com/sites/default/files/ZOE-M8_DataSheet_%28UBX-16008094%29.pdf

    Initalize the data and chip select pins:
    The values will be filled soon
    */
   velocity = 0;
   az = 0;
   altitude = 0;
   roll_rate = 0; 
   lattitude = 0;
   longitude = 0;
   
}
//Thread to log information to BeagleBone. This will be done after data is read.
static THD_FUNCTION(loggerThread, arg) {
    (void)arg;
    //Fastest Baud Rate Possible (Teensy should be able to handle it but can the BBB?)
    Serial.begin(4608000);
    //Sending data in alphabetical order. First 4 bytes is altitude,  second 4 bytes is az, etc.
    float sensorData[6] = {altitude, az, lattitude, longitude, roll_rate, velocity};
    //Creates a byte array of length 24
    byte *data_asByteArray = (byte*)sensorData;
    Serial.write(data_asByteArray, sizeof(data_asByteArray));
}
//Thread for actuating servos for active control
static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bototom
    while(true) {
        if(servo1.attached() == false || servo2.attached() == false || servo3.attached() == false || servo4.attached() == false) {
            delay(TERMINATE); //basically end the program
        }
    }
}
void testServos() {
    currentAngle = initialAngle;
    //For loops test the functionality of the servo's movements;
    for (currentAngle; currentAngle <= initialAngle + 90; currentAngle++) {
        servo1.write(currentAngle);
        servo2.write(currentAngle);
    }
    for (currentAngle; currentAngle >= initialAngle; currentAngle--) {
        servo1.write(currentAngle);
        servo2.write(currentAngle);
    }

    //Testing second set of servos
    for (currentAngle; currentAngle <= initialAngle + 90; currentAngle++) {
        servo3.write(currentAngle);
        servo4.write(currentAngle);
    }
    for (currentAngle; currentAngle >= initialAngle; currentAngle--) {
        servo3.write(currentAngle);
        servo4.write(currentAngle);
    }
}

void chSetup() {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    testServos();
    //Start servoThread
    chThdCreateStatic(waThread, sizeof(waThread), NORMALPRIO, servoThread, NULL);
    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, dataThread,NULL);
    chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, loggerThread,NULL);

    while(true) {
        //Spawning Thread. We just need to keep it running in order to no lose servoThread
        chThdSleep(100);
    }
}

void setup() {
    //Initialize and start ChibiOS (Technically the first thread)
    chBegin(chSetup);
    while(true) {}
    //Must stay in while true loop to keep the chSetup thread running
    }

void loop() {
    //Could cause problems if used for ChibiOS, 
    //but needed to make teensy happy.
}
