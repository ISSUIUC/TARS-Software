#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>

#define BALL_VALVE_1_PIN 2
#define BALL_VALVE_2_PIN 3

#define HYBRID_PT_1_PIN 20
#define HYBRID_PT_2_PIN 21
#define HYBRID_PT_3_PIN 22

#define DEBUG_LED_1 6
#define DEBUG_LED_2 7
#define DEBUG_LED_3 8
#define DEBUG_LED_4 9

//data struct for FSM to send to ballValve_THD
struct ballValve_Message{
  bool isOpen;
  int timeStamp; //timeStamp is set when open or close command is sent.
  //more data can be added as needed
};

//data struct for hybridPT_THD to send pressure transducer data to FSM
struct pressureData{
  int PT1;
  int PT2;
  int PT3;
  int timeStamp; //timeStamp is set when pressure data is read.
  //more data can be added as needed
};

//create servo objects for the ball valve servos
Servo ballValve1;
Servo ballValve2;

//Anshuk: Where will the FSM go?
//Anshuk: Add provisions for reading other sensor data, and thread for relaying to BeagleBone

//----------------------------------------------------------------
//create thread working areas

static THD_WORKING_AREA(ballValve_WA, 32);
thread_t *ballValve_Pointer;

static THD_WORKING_AREA(hybridPT_WA, 32);
thread_t *hybridPT_Pointer;

//----------------------------------------------------------------
//defining threads

//thread that controls the ball valve servos for the hybrid engine.
static THD_FUNCTION(ballValve_THD, arg){
  ballValve_Message *incomingMessage; //create empty pointer for incoming message from FSM
  
  while(true){
    chMsgWait(); //sleep until message is recieved
    incomingMessage = (ballValve_Message*)chMsgGet(/*TODO: Insert pointer to FSM thread here*/);

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

    chMsgRelease(/*TODO: Insert pointer to FSM thread here*/, (msg_t)&incomingMessage); //releases FSM thread and returns incoming message
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

    chMsgSend(/*TODO: Insert pointer to FSM thread here*/, (msg_t)&outgoingMessage);
  }
}

//----------------------------------------------------------------
//setup thread

void chSetup() {
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