#include <Arduino.h>
#include <ChRt.h>
#include <Servo.h>
#include <SPI.h>

float altitude; //current altitude from altimiter
float az; //Acceleration in the z direction
float lattitude; //current lattitude from gps
float longitude; //current longitude from gps
float roll_rate; //angular velocity in the z direction
float velocity; //current velocity of the rocket


void setup() {
    Serial.begin(4608000);
}
 

void loop() {
  if (Serial.available() > 0) { //serial.available returns the number of bytes in the serial buffer
    byte dataBuffer[24];
    float sensorData[6] = {altitude, az, lattitude, longitude, roll_rate, velocity};
    Serial.readBytes(dataBuffer, 24);
    byte altitude_byte_array[4];
    byte az_byte_array[4];
    byte lattitude_byte_array[4];
    byte longitude_byte_array[4];
    byte rr_byte_array[4];
    byte velocity_byte_array[4];

    //Preliminary code for sensor data reception. Need to implement this in linux c++ system.
    //This is assuming information is sent LSB first.
    for(int i = 0; i < 24; i++) {
      if (i < 4) {
        altitude_byte_array[i % 4] = dataBuffer[i];
      } else if (i < 8) { 
        az_byte_array[i % 4] = dataBuffer[i];
      } else if (i < 12) {
        lattitude_byte_array[i % 4] = dataBuffer[i];
      } else if (i < 16) {
        longitude_byte_array[i % 4] = dataBuffer[i];
      } else if (i < 20) {
        rr_byte_array[i % 4] = dataBuffer[i];
      } else {
        velocity_byte_array[i % 4] = dataBuffer[i];
      }
    }
    altitude = *( (float*) altitude_byte_array ); 
    az = *( (float*) az_byte_array ); 
    lattitude = *( (float*) lattitude_byte_array ); 
    longitude = *( (float*) longitude_byte_array );
    roll_rate = *( (float*) rr_byte_array );
    velocity = *( (float*) velocity_byte_array);
    
    //debug purposes
    //Serial.write(dataBuffer, 16);
    //Serial.println(String(az) + " " + String(roll_rate) + " " + String(alt) + " " + String(vel));
  }