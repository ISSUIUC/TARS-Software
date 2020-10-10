#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>

#define SERVO_PIN 5
#define LED_PIN 13
// SEMAPHORES are used to solve MUTEX-ish
// Count of data records in fifo.
SEMAPHORE_DECL(fifoData, 0);

// Count of free buffers in fifo.
SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

Servo myservo;
myservo.attach(SERVO_PIN);

// Give our thread 32 bytes
static THD_WORKING_AREA(waThread1, 32);

static THD_FUNCTION(secondThread, arg)
{
  (void)arg;
  
  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void mainThread()
{
  
  // start producer thread
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, secondThread, NULL);
  
  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);
  } 
}
void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  chBegin(mainThread);

  while(true) {}
}

void loop() {
  
}