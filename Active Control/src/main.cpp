//Here goes the active control Teensy code. @mht3 do whatever you have to on this script (in your own branch)
#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>
#include <SPI.h>

#define SERVO1_PIN 5
#define SERVO2_PIN 6
#define SERVO3_PIN 7
#define SERVO4_PIN 8
#define TERMINATE 9900000000000

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int initialAngle = 0;
int currentAngle;
int altitude; 

//Gives thread 32 bytes. STATIC, so size will not change.
static THD_WORKING_AREA(waThread, 32);
//May have to change the size
static THD_WORKING_AREA(waThread2, 32);

static THD_FUNCTION(dataThread, arg) {
    (void)arg;
    // start the SPI library:
    SPI.begin();
    /*
    //Need IMU, Pitot, and GPS data from Beaglebone...
    
    Datasheet for IMU: https://www.mouser.com/datasheet/2/389/lsm9ds1-1849526.pdf
    Datasheet for Pressure Sensor: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5611-01BA03&DocType=Data+Sheet&DocLang=English
    Datasheet for Accelerometer: https://www.mouser.com/datasheet/2/348/KX134-1211-Specifications-Rev-1.0-1659717.pdf
    Datasheet for GPS: https://www.u-blox.com/sites/default/files/ZOE-M8_DataSheet_%28UBX-16008094%29.pdf

    Initalize the data and chip select pins:

    */
   
}

static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bototom
    while(true) {
        if(servo1.attached() == false || servo2.attached() == false
        || servo3.attached() == false || servo4.attached() == false) {
            delay(TERMINATE); //basically end the program
        }
        // constrain limits range of angle values to between a and b, and does x if x is inbetween this range.
        //Not sure if we will need constrain yet (maybe if we do roll).
        //float theta = 90;
        //servo1.write(constrain(theta,a,b));
        //servo2.write(constrain(theta,a,b)); 
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


