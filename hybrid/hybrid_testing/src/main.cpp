#include <SD.h>
#include <ChRt.h>
#include <PWMServo.h>

#define BALL_VALVE_1_PIN 5
#define BALL_VALVE_2_PIN 7

// Data logger based on a FIFO to decouple SD write latency from data
// acquisition timing.
//
// The FIFO uses two semaphores to synchronize between tasks.

//------------------------------------------------------------------------------
//Create servo objects for main ball value actuation.
PWMServo ballValve1;
PWMServo ballValve2;

//------------------------------------------------------------------------------
//create 32byte working area for ball valve input thread
static THD_WORKING_AREA(waBallValveInput, 32);
thread_t *ballValveInputPointer;

//create 32byte working area for ball valve control thread
static THD_WORKING_AREA(waBallValveServos, 32);
thread_t *ballValveServosPointer;

//------------------------------------------------------------------------------

static THD_FUNCTION(ballValveInput, arg){
  int servoTarget;

  while(true){
    servoTarget = 180;//TODO: Implement input for servo target here

    chMsgSend(ballValveServosPointer, (msg_t)&servoTarget);
  }
}

//------------------------------------------------------------------------------

static THD_FUNCTION(ballValveServos, arg){
  int* servoTarget; //create an empty int pointer for servoTarget

  while(true){
    chMsgWait(); //wait until synchronous message is recieved
    servoTarget = (int*)chMsgGet(ballValveInputPointer);
    
    //set two ball valve actuation servos to the int servoTarget points to.
    ballValve1.write(*servoTarget);
    ballValve2.write(*servoTarget);

    chMsgRelease(ballValveInputPointer, (msg_t)&servoTarget);
  }
}


//------------------------------------------------------------------------------
// Interval between points in units of 1024 usec on AVR, usec on ARM.
const systime_t intervalTicks = TIME_US2I(1000);
//const systime_t intervalTicks = 1;

//------------------------------------------------------------------------------
// SD file definitions.
const uint8_t sdChipSelect = BUILTIN_SDCARD;
File file;
//------------------------------------------------------------------------------
// Fifo definitions.

// Size of fifo in records.
const size_t FIFO_SIZE = 10000; //assumption

// Counter for real time pressure displays
int i=0;

// Count of data records in fifo.
SEMAPHORE_DECL(fifoData, 0);

// Count of free buffers in fifo.
SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

// Data type for fifo item.
struct FifoItem_t {
  uint32_t usec;
  int value1;
  int value2;
  int value3;
  int error;
};
// Array of fifo items.
FifoItem_t fifoArray[FIFO_SIZE];
//------------------------------------------------------------------------------
// Declare a stack with 32 bytes beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread1, 32);

static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  // Index of record to be filled.
  size_t fifoHead = 0;

  // Count of overrun errors.
  int error = 0;

  systime_t logTimeTicks = chVTGetSystemTime();
  while (true) {
    logTimeTicks += intervalTicks;
    chThdSleepUntil(logTimeTicks);
    // Get a buffer.
    if (chSemWaitTimeout(&fifoSpace, TIME_IMMEDIATE) != MSG_OK) {
      // Fifo full, indicate missed point.
      error++;
      continue;
    }
    FifoItem_t* p = &fifoArray[fifoHead];
    p->usec = micros();
    p->value1 = analogRead(A7);
    p->value2 = analogRead(A6);
    p->value3 = analogRead(A5);
    p->error = error;
    error = 0;

    // Counting up at every data acquisition
    i++;

    // Signal new data.
    chSemSignal(&fifoData);

    // Advance FIFO index.
    fifoHead = fifoHead < (FIFO_SIZE - 1) ? fifoHead + 1 : 0;
  }
}

void mainThread();

//------------------------------------------------------------------------------
void setup() {

  //attach ball valve actuation servos to pins
  ballValve1.attach(BALL_VALVE_1_PIN);
  ballValve2.attach(BALL_VALVE_2_PIN);
  
  //move ball valve servos to their initial state
  ballValve1.write(180);
  ballValve2.write(180);

  analogReadResolution(13);
  Serial.begin(9600);

  // Wait for USB Serial.
  while (!Serial) {}

  // Start kernel
  chBegin(mainThread);
  // Start input pin
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);

  // chBegin() resets stacks and should never return.
  while (true) {}
}
//------------------------------------------------------------------------------

void mainThread() {

  // FIFO index for record to be written.
  size_t fifoTail = 0;

  // Time in micros of last point.
  uint32_t last = 0;

  // Remember errors.
  bool overrunError = false;

  // start producer thread
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  //start ball valve control threads TODO: Figure out if these will run with their current priorities
  ballValveInputPointer = chThdCreateStatic(waBallValveInput, sizeof(waBallValveInput), NORMALPRIO, ballValveInput, NULL);
  ballValveServosPointer = chThdCreateStatic(waBallValveServos, sizeof(waBallValveServos), NORMALPRIO, ballValveServos, NULL);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println(F("Send start message to begin ('a' = 0x61)"));

  // Print sensors to Serial port while waiting for start message
  int q = 0;
  while(true) {

    if (Serial.available()) {
      if (Serial.read() == 0x61) {
        // LED on to indicate recording
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      } else {
        Serial.flush();
      }
    }

    // Don't print every cycle to avoid saturating Serial buffer
    if (q >= 300000) {
      q = 0;
      Serial.print(micros());
      Serial.print("\t");
      Serial.print(analogRead(A7));
      Serial.print("\t");
      Serial.print(analogRead(A6));
      Serial.print("\t");
      Serial.println(analogRead(A5));
    }

    ++q;
  }

  // Open file.
  if (!SD.begin(sdChipSelect)) {
    Serial.println(F("SD begin failed."));
    while (true) {}
  }
  file = SD.open("0218B.CSV", O_CREAT | O_WRITE | O_TRUNC);
  if (!file) {
    Serial.println(F("file open  failed."));
    while (true) {}
  }
  // Throw away input.
  while (Serial.read() >= 0);
  Serial.println(F("Send end message to end ('b' = 0x62)"));

  // SD write loop.
  while (true) {

    if (Serial.available()) {
      if (Serial.read() == 0x62) {
        digitalWrite(LED_BUILTIN, LOW);
        break;
      } else {
        Serial.flush();
      }
    }

    // Wait for next data point.
    chSemWait(&fifoData);

    FifoItem_t* p = &fifoArray[fifoTail];
    if (fifoTail >= FIFO_SIZE) fifoTail = 0;

    // Display real time pressure at ~ 30.03 Hz
    if(i >= 33 && Serial.availableForWrite()){
      i=0;
      //Serial.println(p->value*0.016337764-0.300943842);
      Serial.print(p->usec);
      Serial.print("\t");
      Serial.print(p->value1);
      Serial.print("\t");
      Serial.print(p->value2);
      Serial.print("\t");
      Serial.println(p->value3);

      // char* buf = (char)p;
      //
      // Serial.write(buf, 16);
    }

    // Print interval between points.
    if (last) {
      file.print(p->usec - last);
    } else {
      file.write("NA");
    }
    last = p->usec;
    file.write(',');
    file.print(p->value1);
    file.write(',');
    file.print(p->value2);
    file.write(',');
    file.print(p->value3);
    //file.write(',');
    //file.println(p->error);

    // Remember error.
    if (p->error) overrunError = true;

    // Release record.
    chSemSignal(&fifoSpace);

    // Advance FIFO index.
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
  }
  // Close file, print stats and stop.
  file.close();
  Serial.println(F("Done"));
  Serial.print(F("Unused Thread1 stack: "));
  Serial.println(chUnusedThreadStack(waThread1, sizeof(waThread1)));
  Serial.print(F("Unused main stack: "));
  Serial.println(chUnusedMainStack());
  if (overrunError) {
    Serial.println();
    Serial.println(F("** overrun errors **"));
    Serial.println(F("Increase intervalTicks and/or FIFO_SIZE"));
  }
  while (true) {}
}

void loop() {
  // not used
}
