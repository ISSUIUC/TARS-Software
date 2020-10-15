#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>


#define SERVO_PIN 2
#define LED_PIN 13
#define POT_PIN A0

//create struct to hold potentiometer data for sending between threads
struct message{
  int timestamp;
  int potVal; 
};

Servo myservo;

// Give our threads 32 bytes

//servo thread working area
static THD_WORKING_AREA(waThread1, 32);
thread_t *potThreadPointer;

//LED blink thread working area
static THD_WORKING_AREA(waThread2, 32);
thread_t *servoThreadPointer;

//potentiometer input thread working area
static THD_WORKING_AREA(waThread3, 32);

//------------------------------------------------------
//Defining threads

//thread that takes input from a potentiometer and sends that data to servoThread on a scale of 0-180
static THD_FUNCTION(potThread, arg){
  message potData; //create a message called potData

  while(true){
    potData.potVal = map(analogRead(POT_PIN), 0, 1023, 0 , 180); //read value of potentiometer and remap to 0-180 scale
    
    Serial.println(potData.potVal);

    potData.timestamp = chVTGetSystemTime(); //add timestamp in ticks to message
    chMsgSend(servoThreadPointer, (msg_t)&potData); //sends potData to servoThread and thread goes to sleep until reply
  }
}

//thread that sweeps servo
static THD_FUNCTION(servoThread, arg){
  message* potData; //create an empty pointer for message

  while(true){
    chMsgWait(); //waits to recieve message
    potData = (message*)chMsgGet(potThreadPointer); //set potData pointer to location of the incoming message

    myservo.write(potData->potVal);

    chMsgRelease(potThreadPointer, (msg_t)&potData); //send reply to potThread to release it. Also send back pointer to original message
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

//------------------------------------------------------
//Setup Thread

void chSetup(){
  //I don't really get how this pointer stuff works yet, I just saw it in a tutorial
  //start potentiometer data collection thread
  potThreadPointer = chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, potThread, NULL);
  
  //start servo thread
  servoThreadPointer = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, servoThread, NULL);
  
  //start LED blink thread
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, lightThread, NULL);

  while(true);    
}

//------------------------------------------------------

void setup(){
  myservo.attach(SERVO_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

//open serial port
  Serial.begin(9600);
  
  //start ChibiOS
  chBegin(chSetup);

  while(true);
}




void loop(){
  //not used
}
