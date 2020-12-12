#include <Arduino.h>
#include <ChRt.h>
#include <SPI.h>

#define ZOE_M8_GPSmosi 21 //J4 on sensor 
#define ZOE_M8_GPScs   15 //A2
#define ZOE_M8_GPSsclk 20 //B1
#define ZOE_M8_GPSmiso 14 //J5

float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float roll_rate; //angular velocity in the z direction
float g = 9.81; //Acceleration due to gravity in m/s
float lattitude; //current lattitude from gps
float longitude; //current longitude from gps
float pt1; //Pressure data from struct...
float pt2;
float pt3;
int timeStamp;

// data buffer for gps
uint8_t buf[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//"To prevent the receiver from being
//busy parsing incoming data, the parsing
//process is stoppedafter 50 subsequent bytes containing 0xFF."
//This is in section 11.6.2 of interface manual for read access. Would we change the buffer to 50 then?
char gpsbuf[80];




//May have to change the size
static THD_WORKING_AREA(waThread2, 32);
//Workign area thread for logger
static THD_WORKING_AREA(waThread3, 32);

//Thread to get data from sensors
static THD_FUNCTION(dataThread, arg) {
    /*
    GPS data.
    The maximum transfer rate using SPI on ZOE M8 is 125 kB/s and the maximum SPI clock
    frequency is 5.5 MHz. 
    Datasheet for GPS: https://www.u-blox.com/sites/default/files/ZOE-M8_DataSheet_%28UBX-16008094%29.pdf
    Initalize the data and chip select pins:
    */
   
   
    velocity = 0;
    az = 0;
    roll_rate = 0; 
    lattitude = 0;
    longitude = 0;

    (void)arg;
    // start the SPI library:
    Serial.begin(115200);
     // setup GPS
    pinMode( ZOE_M8_GPSmosi, OUTPUT );
    pinMode( ZOE_M8_GPSmiso, INPUT );
    pinMode( ZOE_M8_GPSsclk, OUTPUT );
    pinMode( ZOE_M8_GPScs  , OUTPUT );
    digitalWrite( ZOE_M8_GPScs, HIGH );
    digitalWrite( ZOE_M8_GPSsclk, LOW );
    SPI.begin();

    while(true) {
        gpsbuf[0] = 0;
 
        //Select GPS
        digitalWrite(ZOE_M8_GPScs, LOW);
    
        //Write 0xFF while reading the MISO line and store each char in gpsbuf
        int i = 0, j =0;
        for(j = 0; j < 80; j++) {
            for(i = 0; i < 8; i++) {
                digitalWrite(ZOE_M8_GPSmosi, ((0xFF >> (7-i)) & 0x01));
                digitalWrite(ZOE_M8_GPSsclk, 1);
                delay(1);
                gpsbuf[j] |= digitalRead(ZOE_M8_GPSmiso) << (7-i);
                digitalWrite(ZOE_M8_GPSsclk, 0);
            }
            //Clean gpsbuf while reading
            if(j != 79) {
                gpsbuf[j+1] = 0;
            }
            Serial.print(gpsbuf[j], HEX);
        }
        //Deselect GPS
        digitalWrite(ZOE_M8_GPScs, HIGH);
    }
}

   
//Thread to log information to BeagleBone. This will be done after data is read.
static THD_FUNCTION(loggerThread, arg) {
    (void)arg;
    //Should do fastest Baud Rate Possible (Teensy should be able to handle it but can the BBB?)
    Serial.begin(115200); //Maximum is 4608000. We will have to test to see how much higher we can go before packets are lost.
    //Sending data in alphabetical order. First 4 bytes is altitude,  second 4 bytes is az, etc.
    float sensorData[10] = {altitude, az, lattitude, longitude, roll_rate, velocity, pt1, pt2, pt3, (float) timeStamp};
    //Creates a byte array of length 24
    byte *data_asByteArray = (byte*)sensorData;
    Serial.write(data_asByteArray, 40);
}

void chSetup() {
    
    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, dataThread,NULL);
    chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, loggerThread,NULL);

    while(true) {
        //Spawning Thread. We just need to keep it running in order to no lose servoThread
        chThdSleep(100);
    }
}

void setup() {
    //Initialize and start ChibiOS (Technically the first thread)
    chBegin(chSetup);
    while(true) {}
    //Must stay in while true loop to keep the chSetup thread running
    }

void loop() {
    //Could cause problems if used for ChibiOS, 
    //but needed to make teensy happy.
}
