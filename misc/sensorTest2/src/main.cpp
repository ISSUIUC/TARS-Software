#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "SparkFunLSM9DS1.h"

LSM9DS1 imu;

#define LSM9_CS_AG	36    // LSM9DS1 Accel/Gyro
#define LSM9_CS_M   37    // LSM9DS1 Magnetometer
#define KX134_CS    10    // KX132
#define ZOEm8_CS    34    // ZOE-M8Q

#define LSM9_WHOAMI_REG 0x0F

#define KX134_WHOAMI_REG 0x12

void setup()
{
  // pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LSM9_CS_AG, OUTPUT);
  pinMode(LSM9_CS_M, OUTPUT);

  digitalWrite(LSM9_CS_AG, HIGH);
  digitalWrite(LSM9_CS_AG, HIGH);
  digitalWrite(LSM9_CS_AG, HIGH);

  Serial.begin(115200);

  while(!Serial){}

  uint8_t val[100];

  Serial.println("Beginning SPI");
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);


  /***************************************************************************/
  /* LSM9DS1 WHOAMI                                                          */

  Serial.println("Getting LSM9DS1 WHOAMI");

  digitalWrite(LSM9_CS_AG, LOW);
  delayMicroseconds(1);

  val[0] = SPI.transfer(0x80 | LSM9_WHOAMI_REG);

  for (int i = 1; i < 11 ; ++i) {

      val[i] = SPI.transfer(0x00);

  }

  delayMicroseconds(1);
  digitalWrite(LSM9_CS_AG, HIGH);

  for (int i = 0; i < 11; ++i) {
      Serial.print("Received: 0x");
      Serial.println(val[i], HEX);
  }

  /***************************************************************************/
  /* KX134 WHOAMI                                                          */

  Serial.println("Getting KX134 WHOAMI");

  digitalWrite(KX134_CS, LOW);
  delayMicroseconds(1);

  val[0] = SPI.transfer(0x80 | KX134_WHOAMI_REG);

  for (int i = 0; i < 11 ; ++i) {

      val[i] = SPI.transfer(0x00);

  }

  delayMicroseconds(1);
  digitalWrite(KX134_CS, HIGH);

  for (int i = 0; i < 11; ++i) {
      Serial.print("Received: 0x");
      Serial.println(val[i], HEX);
  }

}

void loop()
{
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(500);
}
