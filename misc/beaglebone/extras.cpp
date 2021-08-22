/**
Extra info for my original idea on serial to the BBB. Most likely will not use. 
Also included is a thread to write to an sd card.
Most of this stuff is just a mess of my ideas haha
-Matt

#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

#define SERVO1_PIN 3
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 9
//Digital pin 2 for slave_select line
#define TERMINATE 9900000000

const int chip = BUILTIN_SDCARD;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
bool loggingMode = false;
File logFile;

float az;                         //Accelerationin z direction
bool launch = false;              //Tracks if aruduino has detected launch
bool launch_init = false;         //True whenever acceleration exceeds threshold
int launch_az_thresh = 1;         //Minimum acceleration for launch to be detected
int launch_time_thresh = 300;     //Amount of time (ms) acceleration must exceed threshold to detect launch
float launch_time = 0;            //First time acceleration above launch threshold is detected
float burn_timer = 0;             //Measures length of burrn for launch time threshold
bool free_fall = false;           //Tracks if rocket is past burnout
bool free_fall_init = false;      //True whenever negative acceleration exceeds threshold after launch
int free_fall_thresh = 1;         //Minimum negative acceleration for burnout to be detected **should be negative**
int freefall_time_thresh = 200;   //Amount of time (ms) acceleration must exceed threshold to detect burnout
float burnout_time = 0;           //First time negative acceleration exceeding threshold is detected
float coast_timer = 0;            //Measures length of negative acceleration for free fall time threshold
float alt;                        //Altitude measured from altimeter
bool apogee = false;              //Defines the end of the coast phase when true
bool apogee_init = false;         //True whenever current altitude is less than previous measured altitude
float apogee_time = 0;            //First time altitude is detectad as decreasing
int apogee_time_thresh = 200;     //Amount of time rocket must descend for apogee to be detected
float descent_timer = 0;          //Measures length of descent for apogee time threshold
float vel;                        //Velocity data received from the Pi
float des_alt;                    //Desired altitude of the flight, either 2345 ft or 3456 ft
float g = 9.81;                   //Acceleration due to gravity in metres/sec
float native_drag;                //Native drag force, have to see how to calculate this
float flaps_drag;                 //Additional drag force due to the deployed flaps
float m;                          //Mass of the rocket in kilograms
float buffer;                     //Buffer percentage for active drag system
int initialAngle = 0; //The initial angle (vertical position) of the servos. Note that this may not be 90.
int currentAngle;


//Gives thread 32 bytes. STATIC, so size will not change.
static THD_WORKING_AREA(waThread, 32);
//May have to change the size
static THD_WORKING_AREA(waThread2, 32);
//Workign area thread for logger
static THD_WORKING_AREA(waThread3, 32);
static THD_WORKING_AREA(waThread4, 32);

thread_t *servo_tp;
thread_t *data_tp;
thread_t *logger_tp;

struct logMessage{
  String action; //The action that will be taken 
  String mode; //Current mode we're in (ie launched, active drag, coast, apogee, etc)
  float sensorValue; //Value of the specific sensor we're looking at
  unsigned long currentTime; //Currenttime since launch
};

//Thread to get data from sensors
static THD_FUNCTION(dataThread, arg) {
    (void)arg;
    // start the SPI library:
    SPI.begin();
     
}
//Thread to log information to BeagleBone. This will be done after data is read.
static THD_FUNCTION(loggerThread, arg) {
    (void)arg;
    Serial.begin(115200);
    logMessage* message;
    message = (logMessage*)chMsgGet(servo_tp); //set pointer to location of the incoming message
    //char[10] = 
    int message_length = 16;
    char messageBuffer[message_length];
    Serial.write(messageBuffer, message_length);
    chMsgRelease(servo_tp, (msg_t)&message); //send reply to potThread to release it. Also send back pointer to original message

}
//Thread to log information to BeagleBone. This will be done after data is read.
//https://www.pjrc.com/store/sd_adaptor.html
//https://www.arduino.cc/en/Tutorial/LibraryExamples/ReadWrite
static THD_FUNCTION(loggerThread2, arg) {
    (void)arg;
    Serial.begin(9600);
    while (!Serial) {
        //Wait to connect to serial
    }
    Serial.print("Initializing SD card...");
    if (!SD.begin(chip)) {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");
    
    // open the file. 
    logFile = SD.open("AC_Logger.txt", FILE_WRITE);
    
    // if the file opened okay, write to it:
    if (logFile) {
        Serial.print("Writing to AC_Logger.txt...");
        logFile.println("Log test: (Can you read this");
        // close the file:
        logFile.close();
        Serial.println("done.");
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening AC_Logger.txt");
    }
    
    // re-open the file for reading:
    logFile = SD.open("AC_Logger.txt");
    if (logFile) {
        Serial.println("AC_Logger.txt:");
        
        // read from the file until there's nothing else in it:
        while (logFile.available()) {
            Serial.write(logFile.read());
        }
        // close the file:
        logFile.close();
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening AC_Logger.txt");
    }
}


//Thread for actuating servos for active control
static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bototom
    while(true) {
        if(servo1.attached() == false || servo2.attached() == false
        || servo3.attached() == false || servo4.attached() == false) {
            delay(TERMINATE); //basically end the program
        }
        logMessage dragMessage;
        dragMessage.currentTime = millis();
        //dragMessage.
        chMsgSend(logger_tp, (msg_t)&dragMessage); //sends potData to servoThread and thread goes to sleep until reply
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
    servo_tp = chThdCreateStatic(waThread, sizeof(waThread), NORMALPRIO, servoThread, NULL);
    data_tp = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, dataThread,NULL);
    logger_tp = chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, loggerThread,NULL);
    chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, loggerThread2,NULL);

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
**/

