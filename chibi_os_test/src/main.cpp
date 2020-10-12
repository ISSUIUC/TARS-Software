#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>


#define SERVO_PIN 5
#define LED_PIN 13

Servo myservo;

// Give our threads 32 bytes
static THD_WORKING_AREA(waThread1, 32);
static THD_WORKING_AREA(waThread2, 32);

//------------------------------------------------------
//Defining threads

//thread that sweeps servo
static THD_FUNCTION(servoThread, arg){
  (void)arg;
  
  while(true){
    for (int pos = 0; pos <= 180; pos += 1){ // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (int pos = 180; pos >= 0; pos -= 1){ // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
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
  // starting threads
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, servoThread, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, lightThread, NULL);

  while(true);    
}

//------------------------------------------------------

void setup(){
  myservo.attach(SERVO_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //start ChibiOS
  chBegin(chSetup);

  while(true);
}




void loop(){
  //not used
}
