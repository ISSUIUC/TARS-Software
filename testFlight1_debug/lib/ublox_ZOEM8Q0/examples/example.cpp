#include <Arduino.h>
#include <SPI.h>

#include "ZOEM8Q0.hpp"

#define LSM_AG_CS_PIN	36  // LSM9DS1 Accel/Gyro
#define LSM_M_CS_PIN	37	// LSM9DS1 Magnetometer
#define KX_CS_PIN   10     	// KX132
#define GPS_CS_PIN   34     // ZOE-M8Q
#define MS_CS_PIN	18		// MS5611

#define CS_PIN GPS_CS_PIN

// #define WHOAMI_REG 0x0F  // LSM9 Accel/Gyro/Mag
// #define WHOAMI_REG 0x00
// #define WHOAMI_REG 0x13	// KX134
#define WHOAMI_REG 0xA2

ZOEM8Q0 gps = ZOEM8Q0();

void setup()
{
	Serial.begin(115200);
	while (!Serial);

  	// val[0] = SPI.transfer(0x80 | WHOAMI_REG);

	Serial.println("##### BEGINNING SPI #####");

	gps.beginSPI(ZOEM8Q0_CS);

	//calls read bytes function again inside ZOEM8Q0
	while (!gps.update_data()) {
		Serial.println(".");
		delay(500);
	}

	Serial.println("");
	
	bool position_lock = gps.get_position_lock();
	if (position_lock) {
		float latitude = gps.get_latitude();
		float longitude = gps.get_longitude();
		float altitude = gps.get_altitude();
		Serial.println("GPS Data in main: ");
		Serial.print("Latitude: ");
		Serial.println(latitude);
		Serial.print("Longitude: ");
		Serial.println(longitude);
		Serial.print("Altitude: ");
		Serial.println(altitude);
	}
}

void loop(){
	while (!gps.update_data()) {
		Serial.println(".");
		delay(500);
	}
	Serial.println("");
	bool position_lock = gps.get_position_lock();
	if (position_lock) {
		float latitude = gps.get_latitude();
		float longitude = gps.get_longitude();
		float altitude = gps.get_altitude();
		Serial.println("GPS Data in main: ");
		Serial.print("Latitude: ");
		Serial.println(latitude);
		Serial.print("Longitude: ");
		Serial.println(longitude);
		Serial.print("Altitude: ");
		Serial.println(altitude);
	} else {
		Serial.println("Searching...");
	}
	delay(1000);
}