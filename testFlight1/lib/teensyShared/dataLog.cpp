#include <SD.h>

#include "dataLog.h"

void init_dataLog(File* dataFile) {

    char fileName[12];

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

    Serial.println(fileName);
    *dataFile = SD.open(fileName, O_CREAT | O_WRITE | O_TRUNC);
    dataFile->println("ax, ay, az, gx, gy, gz, mx, my, mz, hg_ax, hg_ay, hg_az, latitude, longitude, altitude, rocketState, timeStamp");

}


void logData(File* dataFile, dataStruct_t* data, FSM_State rocketState) {

    char buffer[400];

    char buffer1[100];
    snprintf(buffer1,4, "%f, ", data->ax);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->ay);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->az);
    strcat(buffer, buffer1);

    sprintf(buffer1, "%f, ", data->gx);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->gy);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->gz);
    strcat(buffer, buffer1);

    sprintf(buffer1, "%f, ", data->mx);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->my);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->mz);
    strcat(buffer, buffer1);

    sprintf(buffer1, "%f, ", data->hg_ax);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->hg_ay);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->hg_az);
    strcat(buffer, buffer1);

    sprintf(buffer1, "%f, ", data->latitude);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->longitude);
    strcat(buffer, buffer1);
    sprintf(buffer1, "%f, ", data->altitude);
    strcat(buffer, buffer1);

    switch(rocketState){
        case STATE_INIT:
            strcat(buffer, "STATE_INIT, ");
            break;
        case STATE_IDLE:
            strcat(buffer, "STATE_IDLE, ");
            break;
        case STATE_LAUNCH_DETECT:
            strcat(buffer, "STATE_LAUNCH_DETECT, ");
            break;
        case STATE_BOOST:
            strcat(buffer, "STATE_BOOST, ");
            break;
        case STATE_BURNOUT_DETECT:
            strcat(buffer, "STATE_BURNOUT_DETECT, ");
            break;
        case STATE_COAST:
            strcat(buffer, "STATE_COAST, ");
            break;
        case STATE_APOGEE_DETECT:
            strcat(buffer, "STATE_APOGEE_DETECT, ");
            break;
        case STATE_APOGEE:
            strcat(buffer, "STATE_APOGEE, ");
            break;
        case STATE_DROGUE:
            strcat(buffer, "STATE_DROGUE, ");
            break;
        case STATE_MAIN:
            strcat(buffer, "STATE_MAIN, ");
            break;
        default:
            strcat(buffer, "bruh, ");
            break;
    }

    __itoa(data->timeStamp, buffer1, 10);
    strcat(buffer, buffer1);
    strcat(buffer, "\n");

    //Serial.println(buffer);
    dataFile->print(buffer);

    //Writing line of data to SD card
    dataFile->flush();
}
