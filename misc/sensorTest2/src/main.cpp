#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "SparkFunLSM9DS1.h"

LSM9DS1 imu;

#define CS_PIN	36    // LSM9DS1
// #define CS_PIN   10     // KX132
// #define CS_PIN   34     // ZOE-M8Q

// #define WHOAMI_REG 0x0F
#define WHOAMI_REG 0x12

void setup()
{
  // pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  Serial.begin(115200);

  while(!Serial){}

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Henlo");

  int val[100];

  Serial.println("Beginning SPI");
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  Serial.println("Getting WHOAMI");

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  SPI.transfer(0x80 | WHOAMI_REG);

  for (int i = 0; i < 10 ; ++i) {

      val[i] = SPI.transfer(0x00);

  }

  delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH);

  for (int i = 0; i < 10; ++i) {
      Serial.print("Received: 0x");
      Serial.println(val[i], HEX);
  }

  // digitalWrite(LED_BUILTIN, HIGH);

}

void loop()
{
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(500);
}
