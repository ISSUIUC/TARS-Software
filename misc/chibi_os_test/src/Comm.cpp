//TODO determine use of dedicated variable or a locking system via mutex
//currently uses variable

#include <SPI.h>
#include <ChRt.h>

//either 0 or 1
//intiailize to different values on the two different teensies
int busctl = 1;
// int rdData = 0;
int failDetect = 0;

int cycles = 0;

//TODO get actual pin number
#define heartline 13

//to turn into teensy thread
void thread() {
    pinMode(heartline, INPUT);
    digitalWrite(heartline, HIGH);

    while (true) {
        if (busctl) {

            // rdData = 1;
            digitalWrite(heartline, LOW);
            // pinMode(heartline, OUTPUT);

            //TODO insert any data comms here

            // while (rdData);

            pinMode(heartline, INPUT);
            digitalWrite(heartline, HIGH);
            busctl = 0;
        
        } else {

            if (digitalRead(heartline)) {
                busctl = 1;
                cycles = 0;
            } else {
                cycles++;
                if (cycles >= failThresh) {
                    fail = 1;
                    chThdSleepMicroseconds(100);
                }
            }

        }
    }
}
