#include <Arduino.h>
#include <SPI.h>
#include "KX134-1211.h"

#define LSM_AG_CS_PIN	36    // LSM9DS1 Accel/Gyro
#define LSM_M_CS_PIN	37
#define KX_CS_PIN   10     // KX132
#define GPS_CS_PIN   34     // ZOE-M8Q
#define MS_CS_PIN	18		// MS5611

#define CS_PIN KX_CS_PIN

// #define WHOAMI_REG 0x0F  // LSM9 Accel/Gyro/Mag
#define WHOAMI_REG 0x00
// #define WHOAMI_REG 0x13
// #define WHOAMI_REG 0xA2

void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println();
  Serial.println("Henlo");

  KX134 imu;
  imu.init();
  imu.update_data();


  }

void loop()
{
  //weee
}
