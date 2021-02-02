#include <Arduino.h>
#include <SPI.h>

#include "KX134-1211.h"
#include "kx134_1211_Registers.h"

// #define DEBUG

//constructor
KX134::KX134()
{
    //Do any necessary setup
    x_accel = 0;
    y_accel = 0;
    z_accel = 0;
    init();
}

//Returns the raw binary data
int16_t KX134::get_x_accel_raw() {
    return x_accel;
}
int16_t KX134::get_y_accel_raw() {
    return y_accel;
}
int16_t KX134::get_z_accel_raw() {
    return z_accel;
}

//  Range from -64 to 64 Gs. 511.98 obtained by dividing 32767 (max decimal) by 64 (mag gs)
float KX134::get_x_gforce() {
    int16_t decimal = get_x_accel_raw();
    float gForce = (float)decimal / 511.98;
    return gForce;
}
float KX134::get_y_gforce() {
    int16_t decimal = get_y_accel_raw();
    float gForce = (float)decimal / 511.98;
    return gForce;
}
float KX134::get_z_gforce() {
    int16_t decimal = get_z_accel_raw();
    float gForce = (float)decimal / 511.98;
    return gForce;
}

//  Getters for acceleratoin (m/s^2). Derived from g force
//  Multiply the gForce by 9.8m/s^2 to get acceleration
float KX134::get_x_accel() {
    float gForce = get_x_gforce();
    return gForce * 9.8;
}
float KX134::get_y_accel() {
    float gForce = get_y_gforce();
    return gForce * 9.8;
}
float KX134::get_z_accel() {
    float gForce = get_z_gforce();
    return gForce * 9.8;
}

/*
    Converts the binary from the raw data to decimal form
    < 32767 is positive, > 32767 is negative, -32768 is essentially zero

    0000 0000 0000 0001   --   1
    0000 0000 0000 0000   --   0
    1111 1111 1111 1111   --   -1
    0111 1111 1111 1111   --   32767
    1000 0000 0000 0000   --  -32768 (essentially zero)
    1000 0000 0000 0001   --  -32767
*/

//!binary to decimal not needed because input/output are already binary
//!Conversion may add bugs
// int16_t KX134::binary_to_decimal(int16_t binary) {
//     int16_t temp;
//     int16_t decimal = 0;
//     int16_t base = 1;

//     while (binary > 0) {
//         temp = binary % 10;
//         decimal = decimal + temp * base;
//         binary = binary / 10;
//         base = base * 2;
//     }
//     if (decimal > 32768) {
//         decimal = decimal - 32768;
//         decimal = -(32768 - decimal);
//     } else if (decimal == 32768) {
//         return 0;
//     }
//     return decimal;
// }

void KX134::init()
{
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV32);

    pinMode(KX134_CS_PIN, OUTPUT);

    // Set PC1 bit of CNTL1 register to enable measurements
    digitalWrite(KX134_CS_PIN, LOW);   // select chip
    SPI.transfer(CNTL1);
    SPI.transfer(0x98);
    digitalWrite(KX134_CS_PIN, HIGH);   // de-select chip

}

void KX134::update_data()
{
    uint8_t data[6];

    digitalWrite(KX134_CS_PIN, LOW); // select chip
    delayMicroseconds(1);

    SPI.transfer(0x80 | XOUT_L);

    for (int i = 0; i < 6; ++i) {
        data[i] = SPI.transfer(0x00);   // 0b11111111
    }

    digitalWrite(KX134_CS_PIN, HIGH); // de-select chip

    x_accel = (data[1] << 8) | data[0];
    y_accel = (data[3] << 8) | data[2];
    z_accel = (data[5] << 8) | data[4];

#ifdef DEBUG
    Serial.print(x_accel);
    Serial.print(",\t");
    Serial.print(y_accel);
    Serial.print(",\t");
    Serial.println(z_accel);
#endif

}


/*

<data[1]> 00000000
00000000  <data[0]>
------------------- BITWISE OR
<data[1]> <data[0]>



XOUT_L
XOUT_H
YOUT_L
YOUT_H
ZOUT_L
ZOUT_H

*/
