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

#define LAUNCH_AZ_THRESHOLD   1
#define LAUNCH_TIME_THRESHOLD 300
#define FREE_FALL_THRESHOLD   1
#define FALL_TIME_THRESHOLD   300

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

//data structs and enums that will control the fsm
struct HybridMachine
{
  HybridStates prevState;
  HybridStates currState;
  float        launchTime;
  bool         launched;
  float        coastTime;
  bool         boostEnded;
  // Could also hold data like pressure data or anything else needed by every state
};

enum HybridStates
{
    HYBRID_STATE_Idle,             // sit and do nothing
    HYBRID_STATE_Boost,            // the hybrid engine is freaking going.
    HYBRID_STATE_Coast,            // the hybrid enginer is done
    HYBRID_STATE_ERROR,            // the consistent aborting process
    HYBRID_STATE_StateCount        // number of states in the state machine
};

enum HybridStateTransition
{
    HYBRID_TRANSITION_done,      // we are done with the current state
    HYBRID_TRANSITION_notDone,   // We are not done with the current state
    HYBRID_TRANSITION_autoError, // we had an error with the current state
    HYBRID_TRANSITION_manError,  // we had to trigger an error
    HYBRID_TRANSITION_count      // number of hybrid transistion statuses
};

const HybridStates hybridTransistionStatus[HYBRID_STATE_StateCount][HYBRID_TRANSITION_count]
{
  {
      HYBRID_STATE_Boost,           // we are in IDLE and we are done with it
      HYBRID_STATE_Idle,            // we are in IDLE, but we want to stay here
      HYBRID_STATE_ERROR,       // we are in IDLE, but we already messed up somewhere
      HYBRID_STATE_ERROR      // we are in IDLE, but we figured out we messed up
  },
  {
      HYBRID_STATE_Coast,            // we are in boost, but now the engine is done working
      HYBRID_STATE_Boost,            // we are in boost, and we want to stay here
      HYBRID_STATE_ERROR,        // we are in boost, but there is a problem
      HYBRID_STATE_ERROR       // we are in boost, but we figured out there was an error
  },
  {
      HYBRID_STATE_Coast,            // we are in coast, but now the engine is done working
      HYBRID_STATE_Coast,            // we are in boost, and we want to stay here
      HYBRID_STATE_ERROR,        // we are in boost, but there is a problem
      HYBRID_STATE_ERROR       // we are in boost, but we figured out there was an error
  },
  {
      HYBRID_STATE_ERROR,            // we are in ERROR, and we want to stay here for the rest of the time
      HYBRID_STATE_ERROR,            // we are in ERROR, and we want to stay here for the rest of the time
      HYBRID_STATE_ERROR,            // we are in ERROR, and we want to stay here for the rest of the time
      HYBRID_STATE_ERROR             // we are in ERROR, and we want to stay here for the rest of the time
  }
};

//create servo objects for the ball valve servos
Servo ballValve1;
Servo ballValve2;
HybridMachine hybrid;
bool hybridInitialized = false;

//Anshuk: Where will the FSM go?

void initializeHybrid( HybridMachine hybrid )
{
  hybrid.prevState  = HYBRID_STATE_Idle;
  hybrid.currState  = HYBRID_STATE_Idle;
  hybrid.launchTime = 0;
  hyrbid.launched   = false;
  hybrid.coastTime  = 0;
  hyrbid.boostEnded = false;
}

HybridStateTransition doStateIdle( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;

  // if we receive the command to prop load, we are going to change our transition status,
  // otherwise nothing is going to happen and we will stay at not done and continue to IDLE

  // call to the data collection thingy
  //LAUNCH DETECTION LOGIC:
  bool launch = false;

  if(az > LAUNCH_AZ_THRESHOLD && !hybrid.launched)
  {
    // get the current time and assume launch
    hybrid.launchTime = millis();
    hybrid.launched   = true;
  }
  
  if(az > LAUNCH_AZ_THRESHOLD && hyrbid.launched )
    burn_timer = millis() - hybrid.launchTime;                
    if(burn_timer > LAUNCH_TIME_THRESHOLD)                 
    {
      // we have for sure launched
      launch = true;
    }
  } // if the acceleration is larger than the threshold for a long time
  else if(az < launch_az_thresh && hybrid.launched )
  {
    hybrid.launched = false;
  } // if the acceleration falls below the threshold

  if ( launch )
  {
    retVal = HYBRID_TRANSITION_done;
  } // if we know we have launched, go to the next state

  return retVal;
}
HybridStateTransition doStateBoost( HybridStates currenstState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  bool freeFall = false;
  //This state will basically just be used to tell us that the hybrid is still in the boost state, it will check for aborts, and whether or not the engine is still active

  // call the data collection

  if ( az < FREE_FALL_THRESHOLD && !hybrid.boostEnded )
  {
    hybrid.coastTime  = millis();
    hybrid.boostEnded = true;
  }
  if ( az < FREE_FALL_THRESHOLD && hybrid.boostEnded )
  {
     coast_timer = millis() - hybrid.coastTime;
     if ( coast_timer > FALL_TIME_THRESHOLD )
     {
      freeFall = true;
     }
  } // the free fall has continued for long enough and we will enter the coast phase
  else if ( az > FREE_FALL_THRESHOLD && hybrid.boostEnded )
  {
    hybrid.boostEnded = false;
  } // the free fall acceleration threshold was not crossed for long enough

  if ( freeFall )
  {
    retVal = HYBRID_TRANSITION_done
  } // we have entered free fall

  return retVal;
} // the boost phase
HybridStateTransition doStateCoast( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  // This state will basically just be getting the data off of flash memory and on to the beagle bone
  // Once that is done we can shut down the
}
HybridStateTransition doStateError( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  // This state gets fed to from the abort states, it will likely handle flashing any memory to the beagle bone, and maybe continue to update us on the pressures of the engine
  // This state will never end once we hit it.
}

void executeHybrid( HybridStates currentState )
{

  HybridStateTransition transitionStatus;

    switch ( currentState )
    {
      case HYBRID_STATE_Idle:
        transitionStatus = doStateIdle( currentState );
        break;
      case HYBRID_STATE_Boost:
        transitionStatus = doStateBoost( currentState );
        break;
      case HYBRID_STATE_Coast:
        transitionStatus = doStateCoast( currentState );
        break;
      case HYBRID_STATE_ERROR:
        transitionStatus = doStateError( currentState );
        break;
      default:
        break;
    }

    HybridStates nextState = hybridTransistionStatus[currentState][transitionStatus];

    if ( transitionStatus != HYBRID_TRANSITION_notDone )
    {
      hybrid.currState = nextState;
      hybrid.prevState = currentState;
    }
}

//Anshuk: Add provisions for reading other sensor data, and thread for relaying to BeagleBone

//----------------------------------------------------------------
//create thread working areas

static THD_WORKING_AREA(ballValve_WA, 32);
thread_t *ballValve_Pointer;

static THD_WORKING_AREA(hybridPT_WA, 32);
thread_t *hybridPT_Pointer;

static THD_WORKING_AREA(hybridSM_WA, 32);
thread_t *hybridSM_Pointer;

//----------------------------------------------------------------
//defining threads

//thread that controls the ball valve servos for the hybrid engine.
static THD_FUNCTION(ballValve_THD, arg){
  ballValve_Message *incomingMessage; //create empty pointer for incoming message from FSM
  
  while(true){
    chMsgWait(); //sleep until message is recieved
    incomingMessage = (ballValve_Message*)chMsgGet(hybridSM_Pointer);

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

    chMsgRelease(hybridSM_Pointer, (msg_t)&incomingMessage); //releases FSM thread and returns incoming message
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

    chMsgSend(hybridSM_Pointer, (msg_t)&outgoingMessage);
  }
}

static THD_FUNCTION(hyrbiedSM_THD, arg){
  while ( true )
  {
    if ( !hybridInitialized )
    {
      initializeHybrid( hybrid );
      hybridInitialized = true;
    }
    else
    {
      executeHybrid( currentState );
    }
  }
}

//----------------------------------------------------------------
//setup thread

void chSetup(){
  //start ball valve control thread
  ballValve_Pointer = chThdCreateStatic(ballValve_WA, sizeof(ballValve_WA), NORMALPRIO, ballValve_THD, NULL);

  //start hybrid pressure transducer data aquisition thread
  hybridPT_Pointer = chThdCreateStatic(hybridPT_WA, sizeof(hybridPT_WA), NORMALPRIO, hybridPT_THD, NULL);

  //start hybrid state machine thread
  hybridSM_Pointer = chThdCreateStatic(hyrbidSM_WA, sizeof(hyrbiedSM_WA), NORMALPRIO, hyrbidSM_THD, NULL);

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