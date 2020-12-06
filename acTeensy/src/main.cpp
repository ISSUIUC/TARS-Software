#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SPI.h>

#include "hybridShared.h"
#include "acShared.h"

#define SERVO1_PIN 3
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 9
const unsigned int TERMINATE = 99000000;

#define HYBRID_PT_1_PIN 20
#define HYBRID_PT_2_PIN 21
#define HYBRID_PT_3_PIN 22

#define DEBUG_LED_1 6
#define DEBUG_LED_2 7
#define DEBUG_LED_3 8
#define DEBUG_LED_4 9

#define BALL_VALVE_1_PIN 2
#define BALL_VALVE_2_PIN 3

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4; //Remove the fourth servo for test flight

//create servo objects for the ball valve servos
PWMServo ballValve1;
PWMServo ballValve2;

int currentAngle; //Current angle of the servos. 
int initialAngle = 0; //Initial angle of servos. This may not be zero.
float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float old_alt; //Altitude from previous cycle
float roll_rate; //angular velocity in the z direction
float g = 9.81; //Acceleration due to gravity in m/s
float latitude; //current latitude from gps
float longitude; //current longitude from gps
<<<<<<< HEAD
float Kp; //Proportionality constant for roll control PID
float rr_thresh; //Maximum roll rate that we ignore.
float roll_off_alt; //Altitude when roll control stops and we do active drag
float des_alt; //Final altitude goal
float buffer; //Buffer for the active drag
float m; //Mass of the rocket
=======
float pt1; //Pressure data from struct...
float pt2;
float pt3;
int timeStamp;
>>>>>>> dev/Matt

fsm_struct fsm_states; 

float coast_time_thresh;
float coast_thresh;
float coast_timer;
float descent_timer;
float burn_timer; //Measuring how long the burn happens
float launch_az_thresh; //Minimum vert acc. for launch detection
float launch_time_thresh; //Minimum time to confirm launch
float apogee_time_thresh;
bool launch_init = false;
bool coast_init = false;
bool apogee_init;

//Gives thread 32 bytes. STATIC, so size will not change.
static THD_WORKING_AREA(dataThread_WA, 32);
thread_t *dataThread_Pointer;

//May have to change the size
static THD_WORKING_AREA(bbComm_WA, 32);
thread_t *bbComm_Pointer;

//Working area thread for logger
static THD_WORKING_AREA(servoThread_WA, 32);
thread_t *servoThread_Pointer;

//Working area for ball_valve
static THD_WORKING_AREA(ballValve_WA, 32);
thread_t *ballValve_Pointer;

//Working area for pressure transducers
static THD_WORKING_AREA(hybridPT_WA, 32);
thread_t *hybridPT_Pointer;

//create another thread for FSM
static THD_WORKING_AREA(rocket_FSM_WA, 32);
thread_t *rocket_FSM_Pointer;

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
   pt1 = 0;
   pt2 = 0;
   pt3 = 0;
   timeStamp = 0;
   
}

//Thread to log information to BeagleBone. This will be done after data is read.
static THD_FUNCTION(bbComm_THD, arg) {
    (void)arg;
    //Should do fastest Baud Rate Possible (Teensy should be able to handle it but can the BBB?)
    Serial.begin(115200); //Maximum is 4608000. We will have to test to see how much higher we can go before packets are lost.
    //Sending data in alphabetical order. First 4 bytes is altitude,  second 4 bytes is az, etc.
    float sensorData[10] = {altitude, az, lattitude, longitude, roll_rate, velocity, pt1, pt2, pt3, (float) timeStamp};
    //Creates a byte array of length 24
    byte *data_asByteArray = (byte*)sensorData;
    Serial.write(data_asByteArray, 40);
}

//Thread for actuating servos for active control
static THD_FUNCTION(servoThread, arg) {
    (void)arg;
    //while loop is in place of loop() at the bottom. Anshuk: I don't get why this is here.
    while(true) {
        if(servo1.attached() == false || servo2.attached() == false || servo3.attached() == false || servo4.attached() == false) {
            delay(TERMINATE); //basically end the program. Anshuk: Maybe we should just end this thread rather than the whole program
        }
        
        //To determine whether the rocket should use roll/drag control or not. Only done in the coast phase
        if(fsm_states.rocket_state == 2) {
            //record time using millis
            if((roll_rate > rr_thresh || roll_rate < -rr_thresh) && altitude < roll_off_alt)//If the rocket is rolling below a certain altitude..
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
                float control_vel = sqrt(2*(g+(native_drag/m)*(des_alt-altitude))); //From v^2 - u^2 = 2*a*s equation. THe final velocity is zero,...
                //...u is the desired velocity considering flap drag isn't applied throughout
                if (altitude > des_alt) {
                    //flaps are vertical
                    servo1.write(initialAngle);
                    servo2.write(initialAngle); 
                    servo3.write(initialAngle); 
                }
                else {
                    if (velocity > control_vel*(1+buffer/100))
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

//-----------------------------------------
//Hybrid teensy threads that need to be ASLEEP if hybrid teensy is working
//thread that controls the ball valve servos for the hybrid engine.
static THD_FUNCTION(ballValve_THD, arg){

    ballValve_Message *incomingMessage; //create empty pointer for incoming message from FSM
  
    while(true){

        //if fail safe switch is active, then don't active this thread

        chMsgWait(); //sleep until message is recieved
        //Messsage from FSM: incomingMessage = (ballValve_Message*)chMsgGet(/*TODO: Insert pointer to FSM thread here*/);

        //Anshuk: Once FSM is developed, make conditionals below more involved
        if(incomingMessage->isOpen == false){
            ballValve1.write(180);
            ballValve2.write(180);
            digitalWrite(DEBUG_LED_1, HIGH);
        }
        else if(incomingMessage->isOpen == true){
            ballValve1.write(0);
            ballValve2.write(0);
            digitalWrite(DEBUG_LED_1, LOW);
        }

    //chMsgRelease(/*TODO: Insert pointer to FSM thread here*/, (msg_t)&incomingMessage); //releases FSM thread and returns incoming message
  }
}

//thread that recieves pressure transducer data from the hybrid engine. We should incorporate this into the sensor data thread
static THD_FUNCTION(hybridPT_THD, arg){
    pressureData outgoingMessage;
  
    while(true){

        //if fail safe switch is active, then don't active this thread..?

        outgoingMessage.PT1 = analogRead(HYBRID_PT_1_PIN);
        outgoingMessage.PT2 = analogRead(HYBRID_PT_2_PIN);
        outgoingMessage.PT3 = analogRead(HYBRID_PT_3_PIN);

        outgoingMessage.timeStamp = chVTGetSystemTime();

        //chMsgSend(/*TODO: Insert pointer to FSM thread here*/, (msg_t)&outgoingMessage);
    }
}


//Thread for the rocket status logic
static THD_FUNCTION(rocket_FSM, arg) {
    (void)arg;
    while(true) {
        if(az > launch_az_thresh && launch_init == false)     //If high acceleration is observed in z direction...
        {
            fsm_states.launch_time = millis();                             //...assume launch and store launch time
            launch_init = true;
        }
        
        if(az > launch_az_thresh && launch_init == true && fsm_states.rocket_state != 2)   //If the acceleration continues...
        {
            burn_timer = millis() - fsm_states.launch_time;                //...start measuring the lenght of the burn time
            if(burn_timer > launch_time_thresh)                 //If the acceleration lasts long enough...
            {
                fsm_states.rocket_state = 1;                                    //...the launch has occured
                //Serial.println(String(millis()/1000) + "s: Launch detected");
            }
        }
        
        else if(az < launch_az_thresh && launch_init == true && fsm_states.rocket_state != 2)   //If the acceleration was too brief...
        {
            launch_init = false;                                //...reset the launch detection (the acceleration was just an anomaly)
        }

        //BURNOUT DETECTION LOGIC:
        if(az < coast_thresh && fsm_states.rocket_state == 1 && coast_init == false)    //If large negative acceleration is observed after launch...
        {
            fsm_states.burnout_time = millis();                            //...assume burnout and store time of burnout
            coast_init = true;
        }

        if(az < coast_thresh && coast_init == true && fsm_states.rocket_state != 2)   //If the negative acceleration continues...
        {
            coast_timer = millis() - fsm_states.burnout_time;             //...start measuring the lenght of the coast time
            if(coast_timer > coast_time_thresh)             //If the negative acceleration lasts long enough...  
            {
                fsm_states.rocket_state = 2;                                 //...burnout has occured, and the rocket is now coasting
                //Serial.println(String(millis()/1000) + "s: Burnout detected");
            }
        }

        else if(az > coast_thresh && coast_init == true && fsm_states.rocket_state != 2)   //If the negative acceleration was too brief...
        {
            coast_init = false;                             //...reset the burnout detection (the acceleration was just an anomaly)
        }
        
        //APOGEE DETECTION LOGIC:
        if(velocity < 0 && fsm_states.rocket_state == 2 && apogee_init == false)    //If velocity is negative during free fall...      
        {
            fsm_states.apogee_time = millis();                             //...assume apogee and store time of apogee
            apogee_init = true;
        }
        if(velocity < 0 && apogee_init == true && fsm_states.rocket_state != 3)       //If descent continues...
        {
            descent_timer = millis() - fsm_states.apogee_time;             //...start measuring time since apogee
            if(descent_timer > apogee_time_thresh)              //If time since apogee exceeds a threshold...
            {
                fsm_states.rocket_state = 3;                                    //...apogee has occured
            }
        }
        else if(velocity > 0 && apogee_init == true && fsm_states.rocket_state != 3)  //If velocity is positive again...
        {
            apogee_init = false;                               //...reset the apogee detection
        }
    }
}
//--------------------------------------------------------

//Is this being used?
void testServos() {
    //For loops test the functionality of the servo's movements;
    for (currentAngle = initialAngle; currentAngle <= initialAngle + 90; currentAngle++) {
        servo1.write(currentAngle);
        servo2.write(currentAngle);
    }
    for (currentAngle = initialAngle; currentAngle >= initialAngle; currentAngle--) {
        servo1.write(currentAngle);
        servo2.write(currentAngle);
    }

    //Testing second set of servos
    for (currentAngle = initialAngle; currentAngle <= initialAngle + 90; currentAngle++) {
        servo3.write(currentAngle);
        servo4.write(currentAngle);
    }
    for (currentAngle = initialAngle; currentAngle >= initialAngle; currentAngle--) {
        servo3.write(currentAngle);
        servo4.write(currentAngle);
    }
}

void chSetup() {
    ballValve1.attach(BALL_VALVE_1_PIN);
    ballValve2.attach(BALL_VALVE_2_PIN);
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    testServos();
    //Start servoThread
    servoThread_Pointer = chThdCreateStatic(servoThread_WA, sizeof(servoThread_WA), NORMALPRIO, servoThread, NULL);
    dataThread_Pointer = chThdCreateStatic(dataThread_WA, sizeof(dataThread_WA), NORMALPRIO, dataThread,NULL);
    bbComm_Pointer = chThdCreateStatic(bbComm_WA, sizeof(bbComm_WA), NORMALPRIO, bbComm_THD,NULL);
    hybridPT_Pointer = chThdCreateStatic(hybridPT_WA, sizeof(hybridPT_WA), NORMALPRIO, hybridPT_THD, NULL);
    ballValve_Pointer = chThdCreateStatic(ballValve_WA, sizeof(ballValve_WA), NORMALPRIO, ballValve_THD, NULL);
    rocket_FSM_Pointer = chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, NULL);

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
