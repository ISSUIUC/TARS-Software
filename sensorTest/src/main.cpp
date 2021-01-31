#include <Arduino.h>
#include <SPI.h>

#include "KX134-1211.h"

KX134 imu;

void setup(){

    Serial.begin(115200);
    while(!Serial){}
    Serial.println();
    Serial.println("Henlo");

}

void loop()
{
    imu.update_data();

    int16_t ax = imu.get_x_gforce();
    int16_t ay = imu.get_y_gforce();
    int16_t az = imu.get_z_gforce();

    Serial.print("X: ");
    Serial.print(ax);
    Serial.print("\t\t Y: ");
    Serial.print(ay);
    Serial.print("\t\t Z: ");
    Serial.println(az);

    delay(50);

}
