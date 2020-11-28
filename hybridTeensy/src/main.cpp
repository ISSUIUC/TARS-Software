#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>

#include "teensyShared.h"

#define BALL_VALVE_1_PIN 2
#define BALL_VALVE_2_PIN 3

#define HYBRID_PT_1_PIN 20
#define HYBRID_PT_2_PIN 21
#define HYBRID_PT_3_PIN 22

#define DEBUG_LED_1 6
#define DEBUG_LED_2 7
#define DEBUG_LED_3 8
#define DEBUG_LED_4 9

//create servo objects for the ball valve servos
PWMServo ballValve1;
PWMServo ballValve2;

//Anshuk: TODO:Add provisions for reading other sensor data

//----------------------------------------------------------------
//create thread working areas

static THD_WORKING_AREA(fsm_WA, 32);
thread_t *fsm_Pointer;

static THD_WORKING_AREA(bbComm_WA, 32);
thread_t *bbComm_Pointer;

static THD_WORKING_AREA(ballValve_WA, 32);
thread_t *ballValve_Pointer;

static THD_WORKING_AREA(hybridPT_WA, 32);
thread_t *hybridPT_Pointer;

//----------------------------------------------------------------
//defining threads

//FSM Thread
static THD_FUNCTION(fsm_THD, arg){
  //ballValve_THD is waiting for message from FSM
  //hybridPT_THD is sending a message to FSM
  while(true){
    //TODO: implement FSM
  }
}

//thread for communicating with BeagleBone Black
static THD_FUNCTION(bbComm_THD, arg){
  while(true){
    //TODO: implement BeagleBone communication
  }
}

//thread that controls the ball valve servos for the hybrid engine.
static THD_FUNCTION(ballValve_THD, arg){
  ballValve_Message *incomingMessage; //create empty pointer for incoming message from FSM
  
  while(true){
    chMsgWait(); //sleep until message is recieved
    incomingMessage = (ballValve_Message*)chMsgGet(fsm_Pointer);

    //Anshuk: Once FSM is developed, make conditionals below more 
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

    chMsgRelease(fsm_Pointer, (msg_t)&incomingMessage); //releases FSM thread and returns incoming message
  }
}

//thread that recieves pressure transducer data from the hybrid engine.
static THD_FUNCTION(hybridPT_THD, arg){
  pressureData outgoingMessage;
  
  while(true){
    outgoingMessage.PT1 = analogRead(HYBRID_PT_1_PIN);
    outgoingMessage.PT2 = analogRead(HYBRID_PT_2_PIN);
    outgoingMessage.PT3 = analogRead(HYBRID_PT_3_PIN);

    outgoingMessage.timeStamp = chVTGetSystemTime();

    chMsgSend(fsm_Pointer, (msg_t)&outgoingMessage);
  }
}

//----------------------------------------------------------------
//setup thread

void chSetup(){
  //start FSM thread
  fsm_Pointer = chThdCreateStatic(fsm_WA, sizeof(fsm_WA), NORMALPRIO, fsm_THD, NULL);//TODO: Tweak priority

  //start BeagleBone communication thread
  bbComm_Pointer = chThdCreateStatic(bbComm_WA, sizeof(bbComm_WA), NORMALPRIO, bbComm_THD, NULL);
  
  //start ball valve control thread
  ballValve_Pointer = chThdCreateStatic(ballValve_WA, sizeof(ballValve_WA), NORMALPRIO, ballValve_THD, NULL);

  //start hybrid pressure transducer data aquisition thread
  hybridPT_Pointer = chThdCreateStatic(hybridPT_WA, sizeof(hybridPT_WA), NORMALPRIO, hybridPT_THD, NULL);

  while(true);
}

//----------------------------------------------------------------

void setup() {
  ballValve1.attach(BALL_VALVE_1_PIN);
  ballValve2.attach(BALL_VALVE_2_PIN);

  pinMode(DEBUG_LED_1, OUTPUT);
  digitalWrite(DEBUG_LED_1, HIGH);

  //start chibios
  chBegin(chSetup);

  while(true);
}

void loop() {
  //Not used
}