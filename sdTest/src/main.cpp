#include <Arduino.h>
#include <ChRt.h>
#include<SPI.h>
#include<SD.h>

int currentAngle; //Current angle of the servos.
int initialAngle = 0; //Initial angle of servos. This may not be zero.
float velocity; //current velocity of the rocket
float az; //Acceleration in the z direction
float altitude; //current altitude from altimiter
float old_alt; //Altitude from previous cycle
float roll_rate; //angular velocity in the z direction
float g = 9.81; //Acceleration due to gravity in m/s
float latitude; //current latitude from gps
float longitude; //current longitude from gps
float Kp; //Proportionality constant for roll control PID
float rr_thresh; //Maximum roll rate that we ignore.
float roll_off_alt; //Altitude when roll control stops and we do active drag
float des_alt; //Final altitude goal
float buffer; //Buffer for the active drag
float m; //Mass of the rocket
float PT1;
float PT2;
float PT3;
int timeStamp;

//create file object for SD card
File dataFile;
char fileName[12];

static THD_WORKING_AREA(dataThread_WA, 32);
thread_t *dataThread_Pointer;

static THD_FUNCTION(dataThread, arg) {
    (void)arg;
    /*
    Need IMU, Pitot, and GPS data from Beaglebone...

    Datasheet for IMU: https://www.mouser.com/datasheet/2/389/lsm9ds1-1849526.pdf
    Datasheet for Pressure Sensor: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5611-01BA03&DocType=Data+Sheet&DocLang=English
    Datasheet for Accelerometer: https://www.mouser.com/datasheet/2/348/KX134-1211-Specifications-Rev-1.0-1659717.pdf
    Datasheet for GPS: https://www.u-blox.com/sites/default/files/ZOE-M8_DataSheet_%28UBX-16008094%29.pdf

    Initalize the data and chip select pins:
    The values will be filled soon
    */
    while (true) {
        
        velocity = 0;
        az = 0;
        altitude = 0;
        roll_rate = 0;
        latitude = 0;
        longitude = 0;
        PT1 = 0;
        PT2 = 0;
        PT3 = 0;
        timeStamp = 0;

        //string to hold data to write all at once.
        char dataStr[100] = "";
        char buffer[8];

        dtostrf(velocity,6,6,buffer); //convert float to char[]
        strcat(dataStr,buffer); //cat char[] to end of dataStr
        strcat(dataStr,","); //add comma to seperate values

        dtostrf(az,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(altitude,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(roll_rate,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(latitude,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(longitude,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(PT1,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(PT2,6,6,buffer);
        strcat(dataStr,buffer);
        strcat(dataStr,",");

        dtostrf(PT3,6,6,buffer);
        strcat(dataStr,buffer);

        //Writing line of data to SD card
        dataFile = SD.open(fileName, FILE_WRITE);
        dataFile.println(dataStr);
        dataFile.close();
    }
}

void chSetup() {
    
    dataThread_Pointer = chThdCreateStatic(dataThread_WA, sizeof(dataThread_WA), NORMALPRIO, dataThread,NULL);

    while(true) {
        //Spawning Thread. We just need to keep it running in order to no lose servoThread
        chThdSleep(100);
    }
}



void setup() {
    // start the SPI library:
    SPI.begin();
    Serial.begin(9600);

    //SD Card Setup
    if(!SD.begin(BUILTIN_SDCARD)){
        //this is what executes if Teensy fails to communicate with SD card. That is not good. Probably no-go for launch
    }

    strcpy(fileName,"data.csv");

    //checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)){
        bool fileExists = false;
        int i = 1;
        while(fileExists==false){
            if(i > 999){
                //max number of files reached. Don't want to overflow fileName[]. Will write new data to already existing data999.csv
                strcpy(fileName, "data999.csv");
                break;
            }

            //converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            //writes "data(number).csv to fileNameTemp"
            char fileNameTemp[10+strlen(iStr)];
            strcpy(fileNameTemp,"data");
            strcat(fileNameTemp,iStr);
            strcat(fileNameTemp,".csv");

            if(!SD.exists(fileNameTemp)){
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    Serial.println("opening SD");
    Serial.println(fileName);
    //write header for csv file
    dataFile=SD.open(fileName);
    Serial.println("SD opened");
    dataFile.println("velocity,z acceleration,altitude,roll rate,latitude,longitude,PT1,PT2,PT3");
    Serial.println("wrote header");
    dataFile.close();
    Serial.println("closed file");

    Serial.println("starting chibiOS");
    //Initialize and start ChibiOS (Technically the first thread)
    chBegin(chSetup);
    while(true) {}
    //Must stay in while true loop to keep the chSetup thread running
}

void loop() {
  //not used
}