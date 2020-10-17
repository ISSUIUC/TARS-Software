//Here goes the active control Teensy code. @mht3 do whatever you have to on this script (in your own branch)
#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>
#include <SPI.h>

#define SERVO1_PIN 5
#define SERVO2_PIN 6
Servo servo1;
Servo servo2;
int initialAngle = 0;
int currentAngle;
int altitude;
ing 

//Gives thread 32 bytes. STATIC, so size will not change.
static THD_WORKING_AREA(waThread, 32);

static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bototom
    while(true) {
        if(servo1.attached() == false || servo2.attached() == false) {
            delay(9900000000000); //basically end the program
        }
        // constrain limits range of angle values to between a and b, and does x if x is inbetween this range.
        //Not sure if we will need constrain yet (maybe if we do roll).
        //float theta = 90;
        //servo1.write(constrain(theta,a,b));
        //servo2.write(constrain(theta,a,b)); 
    }
}

void chSetup() {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
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

    //Start servoThread
    chThdCreateStatic(waThread, sizeof(waThread), NORMALPRIO, servoThread, NULL);
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


