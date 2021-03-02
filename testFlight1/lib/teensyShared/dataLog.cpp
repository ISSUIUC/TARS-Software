#include <SD.h>

#include "dataLog.h"

/**
 * @brief Creates the name for a file to be written to SD card.
 * 
 * @param fileName Pointer to char[] containing intended name of file. Do not include number or file extension at end of name.
 * @return char* Pointer to inputted char[]. It now contains number (if duplicate file existed) and .csv file extension.
 */
char* name_dataLog(char* fileName) {

    char inputName[strlen(fileName)];

    strcpy(inputName,fileName);

    strcat(fileName,".csv");

    //checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)){
        bool fileExists = false;
        int i = 1;
        while(fileExists==false){
            if(i > 999){
                //max number of files reached. Don't want to overflow fileName[]. Will write new data to already existing data999.csv
                strcat(fileName, inputName);
                strcat(fileName, "999.csv");
                break;
            }

            //converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            //writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName)+strlen(iStr)+6];
            strcat(fileNameTemp, inputName);
            strcat(fileNameTemp,iStr);
            strcat(fileNameTemp,".csv");

            if(!SD.exists(fileNameTemp)){
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    //Serial.println(fileName);
    return fileName;
}

// logData overload for lowg_dataStruct_t
/**
 * @brief Logs low-G IMU data to 1 line of a specified .csv file on SD card.
 * 
 * @param dataFile File on SD card. Object from SD library.
 * @param data Low-G IMU data structure to be logged.
 * @param rocketState Enum containing rocket state to be logged.
 */
void logData(File* dataFile, lowg_dataStruct_t* data, FSM_State rocketState) {

    //TODO: make this just use one print
    dataFile->print(data->ax, 4);
    dataFile->print(",");
    dataFile->print(data->ay, 4);
    dataFile->print(",");
    dataFile->print(data->az, 4);
    dataFile->print(",");
    dataFile->print(data->gx, 4);
    dataFile->print(",");
    dataFile->print(data->gy, 4);
    dataFile->print(",");
    dataFile->print(data->gz, 4);
    dataFile->print(",");
    dataFile->print(data->mx, 4);
    dataFile->print(",");
    dataFile->print(data->my, 4);
    dataFile->print(",");
    dataFile->print(data->mz, 4);
    dataFile->print(",");

    dataFile->print(rocketState);
    dataFile->print(",");
    
    dataFile->print(data->timeStamp);
    dataFile->print("\n");

    //Writing line of data to SD card
    dataFile->flush();
}

// logData overload for highg_dataStruct_t
/**
 * @brief Logs high-G IMU data to 1 line of a specified .csv file on SD card.
 * 
 * @param dataFile File on SD card. Object from SD library.
 * @param data High-G IMU data structure to be logged.
 * @param rocketState Enum containing rocket state to be logged.
 */
void logData(File* dataFile, highg_dataStruct_t* data, FSM_State rocketState) {

    //TODO: make this just use one print
    
    //!highG imu data
    dataFile->print(data->hg_ax, 4);
    dataFile->print(",");
    dataFile->print(data->hg_ay, 4);
    dataFile->print(",");
    dataFile->print(data->hg_az, 4);
    dataFile->print(",");

    dataFile->print(rocketState);
    dataFile->print(",");

    dataFile->print(data->timeStamp);
    dataFile->print("\n");

    //Writing line of data to SD card
    dataFile->flush();
}

// logData overload for gps_dataStruct_t
/**
 * @brief Logs GPS data to 1 line of a specified .csv file on SD card.
 * 
 * @param dataFile File on SD card. Object from SD library.
 * @param data GPS data structure to be logged.
 * @param rocketState Enum containing rocket state to be logged.
 */
void logData(File* dataFile, gps_dataStruct_t* data, FSM_State rocketState) {

    //TODO: make this just use one print
    //TODO: log GPS timestamps

    dataFile->print(data->latitude, 6);
    dataFile->print(",");
    dataFile->print(data->longitude, 6);
    dataFile->print(",");
    dataFile->print(data->altitude, 6);
    dataFile->print(",");
    dataFile->print(rocketState);
    dataFile->print(",");
    dataFile->print(data->posLock);
    dataFile->print(",");
    dataFile->print(data->timeStamp);
    dataFile->print("\n");

    //Writing line of data to SD card
    dataFile->flush();
}
