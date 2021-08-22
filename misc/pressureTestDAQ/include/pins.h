#ifndef PINS_H
#define PINS_H

#define SERVO1_PIN 3
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 9

//!ACTUAL PINS ON MK 1: 
/*    
    PT3 = 44
    PT2 = 43
    PT1 = 41

    HYBRID SERVO 1  = 4
    HYBRID SERVO 2  = 5
    AC SERVO 0      = 6
    AC SERVO 1      = 7
*/
#define HYBRID_PT_1_PIN 20
#define HYBRID_PT_2_PIN 21
#define HYBRID_PT_3_PIN 22
#define HYBRID_VENT_PIN 23

#define LED_WHITE   6
#define LED_ORANGE  7
#define LED_RED     8
#define LED_BLUE    9

#define BALL_VALVE_1_PIN 2
#define BALL_VALVE_2_PIN 3

//define magnetometer chip select pin
#define LSM9DS1_M_CS 37
//define accel/gyro chip select pin
#define LSM9DS1_AG_CS 36

// Loadcell pins connect to pins 26 and 27 respectively
#define LOADCELL_DOUT_PIN 26
#define LOADCELL_SCK_PIN 27

#endif