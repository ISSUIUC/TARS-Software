#include <stdio.h>
#include <string.h>

float altitude; //current altitude from altimiter
float az; //Acceleration in the z direction
float lattitude; //current lattitude from gps
float longitude; //current longitude from gps
float roll_rate; //angular velocity in the z direction
float velocity; //current velocity of the rocket

int main() {
    float sensorData[6] = {0.55, 2.50, 40.110558, -88.228333, 3.1415, 9.7235};
    // Allocate memory for read buffer
    unsigned char *dataBuffer = (unsigned char*)sensorData;


    //This isthe order in which data is sent from the teensy!
    //float sensorData[6] = {altitude, az, lattitude, longitude, roll_rate, velocity};

    unsigned char altitude_byte_array[4];
    unsigned char az_byte_array[4];
    unsigned char lattitude_byte_array[4];
    unsigned char longitude_byte_array[4];
    unsigned char rr_byte_array[4];
    unsigned char velocity_byte_array[4];

    //Unpacking respective bytes.
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

    //Converting all from byte arrays (4 bytes each) to floats!
    altitude = *( (float*) altitude_byte_array ); 
    az = *( (float*) az_byte_array ); 
    lattitude = *( (float*) lattitude_byte_array ); 
    longitude = *( (float*) longitude_byte_array );
    roll_rate = *( (float*) rr_byte_array );
    velocity = *( (float*) velocity_byte_array);


    //For debugging
    printf("Altitude: %f\n", altitude);
    printf("Az: %f\n", az);
    printf("Lattitude: %f\n", lattitude);
    printf("Longitude: %f\n", longitude);
    printf("Roll Rate: %f\n", roll_rate);
    printf("Velocity: %f\n", velocity);

    return 0; // success
}