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
    dataFile->println("ax, ay, az, gx, gy, gz, mx, my, mz, hg_ax, hg_ay, hg_az, rocketState, timeStamp");

}


void logData(File* dataFile, dataStruct_t* data, FSM_State rocketState) {

    /*//TODO: make this just use one print
    dataFile->print(data->ax);
    dataFile->print(",");
    dataFile->print(data->ay);
    dataFile->print(",");
    dataFile->print(data->az);
    dataFile->print(",");
    dataFile->print(data->gx);
    dataFile->print(",");
    dataFile->print(data->gy);
    dataFile->print(",");
    dataFile->print(data->gz);
    dataFile->print(",");
    dataFile->print(data->mx);
    dataFile->print(",");
    dataFile->print(data->my);
    dataFile->print(",");
    dataFile->print(data->mz);
    dataFile->print(",");
    //!highG imu data
    dataFile->print(data->hg_ax);
    dataFile->print(",");
    dataFile->print(data->hg_ay);
    dataFile->print(",");
    dataFile->print(data->hg_az);
    dataFile->print(",");

    // dataFile->print(data->pt1);
    // dataFile->print(",");
    // dataFile->print(data->pt2);
    // dataFile->print(",");
    // dataFile->print(data->pt3);
    // dataFile->print(",");
    dataFile->print(rocketState);
    dataFile->print(",");
    dataFile->print(data->timeStamp);
    dataFile->print("\n");*/

    char buffer[100];

    sprintf(buffer, "%f, ", data->ax);
    sprintf(buffer, "%f, ", data->ay);
    sprintf(buffer, "%f, ", data->az);

    sprintf(buffer, "%f, ", data->gx);
    sprintf(buffer, "%f, ", data->gy);
    sprintf(buffer, "%f, ", data->gz);

    sprintf(buffer, "%f, ", data->mx);
    sprintf(buffer, "%f, ", data->my);
    sprintf(buffer, "%f, ", data->mz);

    sprintf(buffer, "%f, ", data->hg_ax);
    sprintf(buffer, "%f, ", data->hg_ay);
    sprintf(buffer, "%f, ", data->hg_az);

    strcat(buffer, "dummystate, ");
    //TODO: add rocket state here

    __itoa(data->timeStamp, buffer, 10);
    
    dataFile->println(buffer);

    //Writing line of data to SD card
    dataFile->flush();
}
