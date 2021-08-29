
#include "ChRt.h"

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

static THD_WORKING_AREA(dataThread_WA, 256);
thread_t *dataThread_Pointer;

static THD_FUNCTION(dataThread, arg) {
    (void)arg;

    while (true) {

        Serial.println("yayeet");
        digitalWrite(LED_BUILTIN, HIGH);
        chThdSleepMilliseconds(50);
        digitalWrite(LED_BUILTIN, LOW);

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

        dataFile.print(velocity);
        dataFile.print(",");
        dataFile.print(az);
        dataFile.print(",");
        dataFile.print(altitude);
        dataFile.print(",");
        dataFile.print(roll_rate);
        dataFile.print(",");
        dataFile.print(latitude);
        dataFile.print(",");
        dataFile.print(longitude);
        dataFile.print(",");
        dataFile.print(PT1);
        dataFile.print(",");
        dataFile.print(PT2);
        dataFile.print(",");
        dataFile.print(PT3);
        dataFile.print(",");
        dataFile.print(timeStamp);
        dataFile.print(",");

        //Writing line of data to SD card
        dataFile.println("ehhlo");
        dataFile.flush();
    }
}

void chSetup() {

    dataThread_Pointer = chThdCreateStatic(dataThread_WA, sizeof(dataThread_WA),
                                           NORMALPRIO, dataThread, NULL);

}



void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    // start the SPI library:
    SPI.begin();
    Serial.begin(9600);
    while (!Serial) {}

    //SD Card Setup
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("SD Begin Failed");
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

    dataFile = SD.open(fileName, O_CREAT | O_WRITE | O_TRUNC);
    Serial.println("SD opened");

    dataFile.println("velocity,z acceleration,altitude,roll rate,latitude,longitude,PT1,PT2,PT3");
    Serial.println("wrote header");

    Serial.println("starting chibiOS");
    //Initialize and start ChibiOS (Technically the first thread)
    chBegin(chSetup);

    while(true) {}
    //Must stay in while true loop to keep the chSetup thread running
}

void loop() {
    chThdSleepMilliseconds(1000);

  //not used
}
