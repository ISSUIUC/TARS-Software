#include <Arduino.h>

float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float roll_rate; //angular velocity in the z direction
float lattitude; //current lattitude from gps
float longitude; //current longitude from gps
float pt1;
float pt2;
float pt3;
int timeStamp;

void setup() {
    Serial1.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);

    //Serial1.println("Hello this is mht3's teensy code");
    //Random floats for testing UART:
    altitude = 937.7238;
    az = 0.8234;
    lattitude = 40.1119; //lat-long coordinates for talbot
    longitude = 88.2282;
    roll_rate = 0.322746;
    velocity = 10.566;
    pt1 = 3.5257;
    pt2 = 1.75356;
    pt3 = 2.8975;
    timeStamp = 75;

    //Sending data in alphabetical order. First 4 bytes is altitude,  second 4 bytes is az, etc.
    float sensorData[10] = {altitude, az, lattitude, longitude, roll_rate, velocity, pt1, pt2, pt3, (float) timeStamp};

    //Creates a byte array of length 24
    byte *data_asByteArray = (byte*)sensorData;
    Serial1.write(data_asByteArray, 40);

    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
