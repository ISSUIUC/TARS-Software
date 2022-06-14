#ifndef PINS_H
#define PINS_H

#define SERVO_CW_PIN 14

#define HYBRID_PT_1_PIN 20
#define HYBRID_PT_2_PIN 21
#define HYBRID_PT_3_PIN 22

#define LED_BLUE 28
#define LED_RED 31
#define LED_ORANGE 30
#define LED_GREEN 29

// define magnetometer chip select pin
#define LSM9DS1_M_CS 3
// define accel/gyro chip select pin
#define LSM9DS1_AG_CS 2
// define barometer chip select pin
#define MS5611_CS 9
// gps chip select pin
// magwired onto servo2 pin because reasons
#define ZOEM8Q0_CS 16

#define H3LIS331DL_CS 5
#define RFM95_CS 41
#define RFM95_INT 40
#define RFM95_RST 20

// /* Pins for testing on breadboard radio
// #define RFM95_CS 10
// #define RFM95_RST 15
// #define RFM95_EN 14
// #define RFM95_INT 16
// #define LED 13 // Blinks on receipt, nonessential
// */

// kx chip select pin
#define KX134_CS 4


#endif
