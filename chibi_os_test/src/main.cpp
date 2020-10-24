#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>

//comment this out when we are not debugging
#define HYBRID_DEBUG

struct HybridMachine
{
  HybridStates prevState;
  HybridStates currState;
  // Could also hold data like pressure data or anything else needed by every state
};

HybridMachine hybrid;

enum HybridStates
{
    HYBRID_STATE_Idle,             // sit and do nothing
    HYBRID_STATE_PropLoading,      // load properllant onto the rocket
    HYBRID_STATE_PressurizedIdle,  // monitor the pressure
    HYBRID_STATE_IgnitionSequence, // trigger the ingniter and open the ball valve
    HYBRID_STATE_Boost,            // the hybrid engine is freaking going.
    HYBRID_STATE_Coast,            // the hybrid enginer is done
    HYBRID_STATE_ManualAbort,      // We had to abort
    HYBRID_STATE_AutoAbort,        // The computer broke, do something.
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
      HYBRID_STATE_PropLoading,     // we are in IDLE and we are done with it
      HYBRID_STATE_Idle,            // we are in IDLE, but we want to stay here
      HYBRID_STATE_AutoAbort,       // we are in IDLE, but we already messed up somewhere
      HYBRID_STATE_ManualAbort      // we are in IDLE, but we figured out we messed up
  },
  {
      HYBRID_STATE_PressurizedIdle, // we are loading prop, and now we just want to chill
      HYBRID_STATE_PropLoading,     // we are loading prop, and we are still loading prop
      HYBRID_STATE_AutoAbort,       // we are loading prop, and an error was detected
      HYBRID_STATE_ManualAbort      // we are loading prop, and we need to stop everything
  },
  {
      HYBRID_STATE_IgnitionSequence, // we are monitoring pressure, and we are clear to launch
      HYBRID_STATE_PressurizedIdle,  // we are monitoring pressure, and we want to keep doing that
      HYBRID_STATE_AutoAbort,        // we are monitoring pressure, and we apparently need to stop
      HYBRID_STATE_ManualAbort       // we are monitoring pressure, and we know we need to stop
  },
  {
      HYBRID_STATE_Boost,            // we ignited, and now we are going
      HYBRID_STATE_IgnitionSequence, // we ignited, and we for some reason are still there( should not happen )
      HYBRID_STATE_AutoAbort,        // we ignited, and we need to stop
      HYBRID_STATE_ManualAbort       // we ignited, but we told it we need to stop
  },
  {
      HYBRID_STATE_Coast,            // we are in boost, but now the engine is done working
      HYBRID_STATE_Boost,            // we are in boost, and we want to stay here
      HYBRID_STATE_AutoAbort,        // we are in boost, but there is a problem
      HYBRID_STATE_ManualAbort       // we are in boost, but we figured out there was an error
  },
  {
      HYBRID_STATE_Coast,            // we are in coast, but now the engine is done working
      HYBRID_STATE_Coast,            // we are in boost, and we want to stay here
      HYBRID_STATE_AutoAbort,        // we are in boost, but there is a problem
      HYBRID_STATE_ManualAbort       // we are in boost, but we figured out there was an error
  },
  {
      HYBRID_STATE_ERROR,            // we are in manualAbort, and we need to follow the consistent shut down process
      HYBRID_STATE_ManualAbort,      // we are in manualAbort, but that hasn't finished yet
      HYBRID_STATE_AutoAbort,        // we are in manualAbort, but um...
      HYBRID_STATE_ManualAbort       // we are in manualAbort, but we needed to abort some more
  },
  {
      HYBRID_STATE_ERROR,            // we are in autoAbort, but we need to follow the consistent shut down process
      HYBRID_STATE_AutoAbort,        // we are in autoAbort, but we are not done with that yet
      HYBRID_STATE_AutoAbort,        // if this happens, we really f--ed it up somewhere
      HYBRID_STATE_ManualAbort       // we are in autoAbort, but we paniced and spammed the stop button
  },
  {
      HYBRID_STATE_ERROR,            // we are in ERROR, and we want to stay here for the rest of the time
      HYBRID_STATE_ERROR,            // we are in ERROR, and we want to stay here for the rest of the time
      HYBRID_STATE_ERROR,            // we are in ERROR, and we want to stay here for the rest of the time
      HYBRID_STATE_ERROR             // we are in ERROR, and we want to stay here for the rest of the time
  }
};

void initializeHybrid( void )
{
  hybrid.prevState = HYBRID_STATE_Idle;
  hybrid.currState = HYBRID_STATE_Idle;
}

HybridStateTransition doStateIdle( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;

  // if we receive the command to prop load, we are going to change our transition status,
  // otherwise nothing is going to happen and we will stay at not done and continue to IDLE
}
HybridStateTransition doStatePropLoading( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;

  //we are going to either check how prop loading is doing(whether that be receiving a value or a bool, or reading a sensor)
  // then only when we hit that will we change to done
  // we are also going to need to be checking the pressue for anamolies, in case we need to abort

}
HybridStateTransition doStatePressurizedIdle( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  // presurized IDLE will just be checking the pressue ofthe engine, and checking for autoAbort, and sending values for manual abort
  // in order to exit the state, we either need to hit an abort or we need to receive the ignition sequence
}
HybridStateTransition doStateIgnitionSequence( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  // This state will need to happen in 1 go(for timing reasons(at least that is the intent)).
  // This state will involve lighting the igniter(will be done by ground station(most likely)) and turning the ball valve, will be done by us
  // we will need to synchronize the two with some sort of circuit or something
  // exits of this state will be either something measured by the pressure sensors, or messages from the IMU
  // aborts can happen, but they will probably have different conditions than autoAborts for the previous states
}
HybridStateTransition doStateBoost( HybridStates currenstState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  //This state will basically just be used to tell us that the hybrid is still in the boost state, it will check for aborts, and whether or not the engine is still active
}
HybridStateTransition doStateCoast( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  // This state will basically just be getting the data off of flash memory and on to the beagle bone
  // Once that is done we can shut down the 
}
HybridStateTransition doStateManualAbort( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  //This state will handle any manual abort, it will read the previous state and know what to do based on that
  // THis state will lead to the ERROR state
}
HybridStateTransition doStateAutoAbort( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  //This state will handle any auto abort, it will read the previos state and know what to do based on that
  // THis state will exit to the ERROR state
}
HybridStateTransition doStateError( HybridStates currentState )
{
  HybridStateTransition retVal = HYBRID_TRANSITION_notDone;
  // This state gets fed to from the abort states, it will likely handle flashing any memory to the beagle bone, and maybe continue to update us on the pressures of the engine
  // This state will never end once we hit it.
}
HybridStates executeHybrid( HybridStates currentState )
{

  HybridStateTransition transitionStatus;

    switch ( currentState )
    {
      case HYBRID_STATE_Idle:
        transitionStatus = doStateIdle( currentState );
        break;
      case HYBRID_STATE_PropLoading:
        transitionStatus = doStatePropLoading( currentState );
        break;
      case HYBRID_STATE_PressurizedIdle:
        transitionStatus = doStatePressurizedIdle( currentState);
        break;
      case HYBRID_STATE_IgnitionSequence:
        transitionStatus = doStateIgnitionSequence( currentState );
        break;
      case HYBRID_STATE_Boost:
        transitionStatus = doStateBoost( currentState );
        break;
      case HYBRID_STATE_Coast:
        transitionStatus = doStateCoast( currentState );
        break;
      case HYBRID_STATE_ManualAbort:
        transitionStatus = doStateManualAbort( currentState );
        break;
      case HYBRID_STATE_AutoAbort:
        transitionStatus = doStateAutoAbort( currentState );
        break;
      case HYBRID_STATE_ERROR:
        transitionStatus = doStateError( currentState );
        break;
      default:
        break;
    }

    HybridStates nextState = hybridTransistionStatus[currentState][transitionStatus];
    //TODO serial print the transition
    if ( transitionStatus == HYBRID_TRANSITION_done )
    {
      hybrid.prevState = currentState;
      hybrid.currState = nextState;
    }
    return nextState;
}
#ifdef HYBRID_DEBUG
const char* hybridTransistionMessages[HYBRID_STATE_StateCount][HYBRID_TRANSITION_count]
{
  {
      "We are in IDLE and we are going to load prop",     
      "We are in IDLE and we are going to stay in IDLE",   
      "We are in IDLE and we are autoAborting",     
      "We are in IDLE and we are manualAborting"     
  },
  {
      "We are loading prop and we are going to monitor pressure", 
      "We are loading prop and we are going to keep monitor pressure",     
      "We are loading prop and we are going to autoAbort",       
      "We are loading prop and we are going to manualAbort"      
  },
  {
      "We are monitoring pressure and we are going to boom ", 
      "We are monitoring pressure and we are going to continue that", 
      "we are monitoring pressure and we are need to autoAbort",
      "we are monitoring pressure and we are going to manualAbort"
  },
  {
      "we are igniting and we are going to boost",
      "we are igniting and we are going to stay there(bad news)",
      "we are igniting and we are going to autoAbort",
      "we are igniting and we are going to manualAbort"
  },
  {
      "we are boosting, but now are are done :(",
      "we are boosting, but we are going to continue",
      "we are boosting, and we are going to do an autoabort",
      "we are boosting, and we are going to do a manualAbort"
  },
  {
      "we are coasting, but we are going to stay in coast forever if possible",
      "we are coasting, but we are going to stay here",
      "we are coasting, but we are going to do an autoAbort",
      "we are coasting, but we are going to do a manualAbort"
  },
  {
      "we are in a manualAbort, and we need to go to ERROR forever",
      "we are in a manualAboer, and we are going to stay there",
      "we are in a manualAbort, and we are um...",
      "we are in a manualAboer, and we are going to autoAbort?"
  },
  {
      "we are in an autoAboer, and we are going to go to ERROR forever",
      "we are in an autoAbortm and we are going to stay in autoAbort",
      "we are in an autoAbort and we are going to do another one?",
      "we are in an autoAboert and we are spamming the stop button"
  },
  {
      "we are in ERROR, so we stay here", 
      "we are in ERROR, so we stay here",
      "we are in ERROR, so we stay here",
      "we are in ERROR, so we stay here"
  }
};
#endif

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}