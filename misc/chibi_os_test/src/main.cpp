#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>


#define SERVO1_PIN 2
#define SERVO2_PIN 3
#define LED_PIN 13
#define POT_PIN A0

//create struct to hold potentiometer data for sending between threads
struct potMessage{
  int timestamp;
  int potVal; 
};

//create struct to hold user input data from serial from serialInThread to servo2Thread
struct serialMessage{
  int timestamp;
  bool servoOpen;
};

//create servo objects
Servo servo1;
Servo servo2;

// Give our threads 32 bytes

//pot thread working area
static THD_WORKING_AREA(waThread1, 32);
thread_t *potThreadPointer;

//servo1Thread working area
static THD_WORKING_AREA(waThread2, 32);
thread_t *servo1ThreadPointer;

//LED blink thread working area
static THD_WORKING_AREA(waThread3, 32);

//serialInThread working area
static THD_WORKING_AREA(waThread4, 32);
thread_t *serialInThreadPointer;

//servo2Thread working area
static THD_WORKING_AREA(waThread5, 32);
thread_t *servo2ThreadPointer;

//------------------------------------------------------
//Defining threads

//thread that takes input from a potentiometer and sends that data to servoThread on a scale of 0-180
static THD_FUNCTION(potThread, arg){
  potMessage potData; //create a message called potData

  while(true){
    potData.potVal = map(analogRead(POT_PIN), 0, 1023, 0 , 180); //read value of potentiometer and remap to 0-180 scale

    potData.timestamp = chVTGetSystemTime(); //add timestamp in ticks to message
    chMsgSend(servo1ThreadPointer, (msg_t)&potData); //sends potData to servoThread and thread goes to sleep until reply
  }
}


//thread that controls servo1
static THD_FUNCTION(servo1Thread, arg){
  potMessage* potData; //create an empty pointer for potMessage

  while(true){
    chMsgWait(); //waits to recieve potMessage
    potData = (potMessage*)chMsgGet(potThreadPointer); //set potData pointer to location of the incoming potMessage

    servo1.write(potData->potVal);

    chMsgRelease(potThreadPointer, (msg_t)&potData); //send reply to potThread to release it. Also send back pointer to original potMessage
  }
}


//thread that blinks light
static THD_FUNCTION(lightThread, arg){
  (void)arg;

  while(true){
    digitalWrite(LED_BUILTIN, HIGH); //turn on built in LED by powering pin 13
    chThdSleep(10); //put thread to sleep while not doing anything
    digitalWrite(LED_BUILTIN, LOW); //turn off built in LED by powering off pin 13
    chThdSleep(300);
    digitalWrite(LED_BUILTIN, HIGH);
    chThdSleep(10);
    digitalWrite(LED_BUILTIN, LOW);
    chThdSleep(1000);
  }
}


//thread that takes input (0 or 1 int) from serial and sends that data to servo2Thread
static THD_FUNCTION(serialInThread, arg){
  serialMessage userInputData; //create a potMessage called potData

  int tempVal = 0;
  Serial.println("Enter \"1\" to open valve. Enter \"0\" to close valve.");

  while(true){
    if(Serial.available() > 0){
      
      tempVal = Serial.parseInt();

      if(tempVal == 1){
        userInputData.servoOpen = true;
        Serial.println("open");
        userInputData.timestamp = chVTGetSystemTime();
      }
      if(tempVal == 0){
        userInputData.servoOpen = false;
        Serial.println("closed");
        userInputData.timestamp = chVTGetSystemTime();
      }
      
      chMsgSend(servo2ThreadPointer, (msg_t)&userInputData);
    }
  }
}


static THD_FUNCTION(servo2Thread, arg){
  serialMessage* userInputData; //create an empty pointer for userInputData

  while(true){
    chMsgWait(); //wait to recieve userInputData
    userInputData = (serialMessage*)chMsgGet(serialInThreadPointer); //set userInputData pointer to the location of the incoming message

    if(userInputData->servoOpen == true){
      servo2.write(0); //open servo
    }
    if(userInputData->servoOpen == false){
      servo2.write(180); //close servo
    }

    chMsgRelease(serialInThreadPointer, (msg_t)&userInputData);
  }
}


//------------------------------------------------------
//Setup Thread

void chSetup(){
  //start potentiometer data collection thread
  potThreadPointer = chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, potThread, NULL);
  
  //start servo1 control thread
  servo1ThreadPointer = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, servo1Thread, NULL);
  
  //start LED blink thread
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO + 1, lightThread, NULL);

  //start serial data collection thread
  serialInThreadPointer = chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, serialInThread, NULL);

  //start servo2 control thread
  servo2ThreadPointer = chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, servo2Thread, NULL);

  while(true);    
}

//------------------------------------------------------

void setup(){
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo2.write(180);//set servo to initial position of closed

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  //open serial port
  Serial.begin(9600);
  Serial.setTimeout(10);
  
  //start ChibiOS
  chBegin(chSetup);

  while(true);
}




void loop(){
  //not used
}
