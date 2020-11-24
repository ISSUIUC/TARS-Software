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
PWMServo servo4; //Remove the fourth servo for test flight


int currentAngle; //Current angle of the servos. 
int initialAngle = 0; //Initial angle of servos. This may not be zero.
float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float roll_rate; //angular velocity in the z direction
float g = 9.81; //Acceleration due to gravity in m/s
float latitude; //current latitude from gps
float longitude; //current longitude from gps
float Kp; //Proportionality constant for roll control PID
float rr_thresh; //Maximum roll rate that we ignore.
float roll_alt_cutoff; //Altitude when roll control stops and we do active drag
float des_alt; //Final altitude goal
float buffer; //Buffer for the active drag


//Gives thread 32 bytes. STATIC, so size will not change.
static THD_WORKING_AREA(waThread, 32);
//May have to change the size
static THD_WORKING_AREA(waThread2, 32);
//Working area thread for logger
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
   latitude = 0;
   longitude = 0;
   
}
//Thread to log information to BeagleBone. This will be done after data is read.
static THD_FUNCTION(loggerThread, arg) {
    (void)arg;
    //Fastest Baud Rate Possible (Teensy should be able to handle it but can the BBB?)
    Serial.begin(9600); //Maximum is 4608000. We will have to test to see how much higher we can go before packets are lost.
    //Sending data in alphabetical order. First 4 bytes is altitude,  second 4 bytes is az, etc.
    float sensorData[6] = {altitude, az, latitude, longitude, roll_rate, velocity};
    //Creates a byte array of length 24
    byte *data_asByteArray = (byte*)sensorData;
    Serial.write(data_asByteArray, sizeof(data_asByteArray));
}
//Thread for actuating servos for active control
static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bottom. Anshuk: I don't get why this is here.
    while(true) {
        if(servo1.attached() == false || servo2.attached() == false || servo3.attached() == false || servo4.attached() == false) {
            delay(TERMINATE); //basically end the program
        
        //To determine whether the rocket should use roll/drag control or not. Maybe use the FSM
        // if(rocket is still burning):
            //record time using millis
        // else if(rocket is in coast phase):
            //record time using millis
            if((roll_rate > rr_thresh || roll_rate < -rr_thresh) && alt < roll_off_alt)//If the rocket is rolling below a certain altitude..
            {
                //...reduce the roll
                //Proportional controller to reduce the roll of the rocket. Kp is the proportional gain.
                float roll_err = -roll_rate; //The desired roll rate is 0, so the error = 0-roll
                float theta = Kp*roll_err; //Make sure the Kp accounts for radian-degree conversion!
        
                //Make sure this block behaves as expected and outputs an angle of theta
                servo1.write(constrain(initialAngle+theta,initialAngle-15,initialAngle+15));
                servo2.write(constrain(initialAngle+theta,initialAngle-15,initialAngle+15)); 
                servo3.write(constrain(initialAngle+theta,initialAngle-15,initialAngle+15));
            }

            else {
                //implement active drag. This needs to be tested
                //control_vel = f(altitude)
                float native_drag; //Need to calculate this using velocity and CD of rocket without flaps
                float control_vel = sqrt(2*(g+(native_drag/m)*(des_alt-alt))); //From v^2 - u^2 = 2*a*s equation. THe final velocity is zero,...
                //...u is the desired velocity considering flap drag isn't applied throughout
                if (alt > des_alt) {
                    //flaps are vertical
                    servo1.write(initialAngle);
                    servo2.write(initialAngle); 
                    servo3.write(initialAngle); 
                }
                else {
                    if (vel > control_vel*(1+buffer/100))
                    {
                        //flaps are deployed
                        servo1.write(initialAngle);
                        servo2.write(initialAngle);
                        servo3.write(initialAngle);
                    }
                    else {
                        //go to vertical
                        servo1.write(initialAngle);
                        servo2.write(initialAngle);
                        servo3.write(initialAngle);
                    }
                } 
            }
  }

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
