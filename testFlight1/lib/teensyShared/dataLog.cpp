#include <SD.h>

#include "dataLog.h"

#include <stdio.h>

#include <string>
#include <cstring>
#include <sstream>

#define THREAD_DEBUG

static THD_FUNCTION(dataLogger_THD, arg){
  struct datalogger_THD *datalogger_struct = (struct datalogger_THD *)arg;
  while(true){
    #ifdef THREAD_DEBUG
        Serial.println("Data Logging thread entrance");
        Serial.println(datalogger_struct->sensor_type);
    #endif
    chSemWait(&datalogger_struct->fifoData);
    chMtxLock(&datalogger_struct->dataMutex);

    sensorDataStruct_t current_data = datalogger_struct->fifoArray[datalogger_struct->fifoTail];
    sensors sensorType = datalogger_struct->sensor_type;

    datalogger_struct->fifoTail = datalogger_struct->fifoTail < (FIFO_SIZE - 1) ? datalogger_struct->fifoTail + 1 : 0;
    chSemSignal(&datalogger_struct->fifoSpace);
    chMtxUnlock(&datalogger_struct->dataMutex);
    
    logData(&datalogger_struct->dataFile, &current_data, sensorType);

    #ifdef THREAD_DEBUG
        Serial.println(datalogger_struct->sensor_type);
        Serial.println("Data Logging thread exit");
    #endif

    chThdSleepMilliseconds(6);
  }
}



/**
 * @brief Creates the name for a file to be written to SD card.
 * 
 * @param fileName Pointer to char[] containing intended name of file. Do not include number or file extension at end of name. Make sure this is longer than it needs to be.
 * @param fileExtension Pointer to char[] containing the file extension for the file.
 * @return char* Pointer to inputted char[]. It now contains number (if duplicate file existed) and .csv file extension.
 */
char* sd_file_namer(char* fileName, char* fileExtensionParam) {
    
    char fileExtension[strlen(fileExtensionParam)+1];
    strcpy(fileExtension, fileExtensionParam);
    
    char inputName[strlen(fileName)+1];
    strcpy(inputName,fileName);
    
    strcat(fileName,fileExtension);    

    //checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)){
        bool fileExists = false;
        int i = 1;
        while(fileExists==false){
            if(i > 999){
                //max number of files reached. Don't want to overflow fileName[]. Will write new data to already existing data999.csv
                strcpy(fileName, inputName);
                strcat(fileName, "999");
                strcat(fileName, fileExtension);
                break;
            }

            //converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            //writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName)+strlen(iStr)+6];
            strcpy(fileNameTemp, inputName);
            strcat(fileNameTemp,iStr);
            strcat(fileNameTemp, fileExtension);

            if(!SD.exists(fileNameTemp)){
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    // Serial.println(fileName);
    return fileName;
}


/**
 * @brief Logs data to 1 line of a specified .csv file on SD card.
 * 
 * @param dataFile File on SD card. Object from SD library.
 * @param data Data structure to be logged.
 * @param sensorType Enum containing the type of sensor. Controls which fields are logged.
 */
void logData(File* dataFile, sensorDataStruct_t* data, sensors sensorType) {

    //TODO: make this just use one print
    /* switch (sensorType) {
        case LOWG_IMU:
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
            
            dataFile->print(data->timeStamp);
            dataFile->print("\n");
            break;
        case HIGHG_IMU:
            dataFile->print(data->hg_ax, 4);
            dataFile->print(",");
            dataFile->print(data->hg_ay, 4);
            dataFile->print(",");
            dataFile->print(data->hg_az, 4);
            dataFile->print(",");

            // dataFile->print(rocketState);
            // dataFile->print(",");

            dataFile->print(data->timeStamp);
            dataFile->print("\n");
            break;
        case GPS:
            dataFile->print(data->latitude, 6);
            dataFile->print(",");
            dataFile->print(data->longitude, 6);
            dataFile->print(",");
            dataFile->print(data->altitude, 6);
            dataFile->print(",");
            // dataFile->print(rocketState);
            // dataFile->print(",");
            dataFile->print(data->posLock);
            dataFile->print(",");
            dataFile->print(data->timeStamp);
            dataFile->print("\n");
            break;
    } */

    // For more efficient printing to file (WIP)

    dataFile->println(formatString(data,sensorType));

    // free(buffer_string);

    //Writing line of data to SD card
    dataFile->flush();
}

char dataChar[100];
char* formatString(sensorDataStruct_t* data, sensors sensorType) {
    /* std::string buffer_string = std::string(1,data->ax);
    buffer_string.append(", ");
    buffer_string.append(str(data->ay));
    buffer_string.append(", ");
    buffer_string.append(str(data->az));
    buffer_string.append(", ");
    buffer_string.append(str(data->gx));
    buffer_string.append(", ");
    buffer_string.append(str(data->gy));
    buffer_string.append(", ");
    buffer_string.append(str(data->gz));
    buffer_string.append(", ");
    buffer_string.append(str(data->mx));
    buffer_string.append(", ");
    buffer_string.append(str(data->my));
    buffer_string.append(", ");
    buffer_string.append(str(data->mz));
    buffer_string.append(", ");
    buffer_string.append(str(data->timeStamp));
    
    return buffer_string.c_str(); */
    /* Serial.println(5);
    std::ostringstream ss;
    switch (sensorType) {
        case LOWG_IMU:
            ss << data->ax;
            ss << ',';
            ss << data->ay;
            ss << ',';
            ss << data->az;
            ss << ',';
            ss << data->gx;
            ss << ',';
            ss << data->gy;
            ss << ',';
            ss << data->gz;
            ss << ',';
            ss << data->mx;
            ss << ',';
            ss << data->my;
            ss << ',';
            ss << data->mz;
            ss << ',';
            ss << data->rocketState;
            ss << ',';
            ss << data->timeStamp;
            break;
        case HIGHG_IMU:
            ss << data->hg_ax;
            ss << ',';
            ss << data->hg_ay;
            ss << ',';
            ss << data->hg_az;
            ss << ',';
            ss << data->rocketState;
            ss << ',';
            ss << data->timeStamp;
            break;
        case GPS:
            ss << data->latitude;
            ss << ',';
            ss << data->longitude;
            ss << ',';
            ss << data->altitude;
            ss << ',';
            ss << data->posLock;
            ss << ',';
            ss << data->rocketState;
            ss << ',';
            ss << data->timeStamp;
            break;
    }
    Serial.println(6);
    const std::string bufferString = ss.str();
    Serial.println(7);
    const char* bufferChar = bufferString.c_str();
    Serial.println(bufferChar);
    return bufferChar; */
    strcpy(dataChar,"");
    Serial.println(5);
    switch (sensorType) {
        case LOWG_IMU:
            Serial.println(data->gx);
            Serial.println(data->rocketState);
            snprintf(dataChar,sizeof(dataChar),"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%i,%ld",
                            data->ax,data->ay,data->az,
                            data->gx,data->gy,data->gz,
                            data->mx,data->my,data->mz,
                            data->rocketState,data->timeStamp);
            Serial.println(dataChar);
            break;
        case HIGHG_IMU:
            snprintf(dataChar,sizeof(dataChar),"%.4f,%.4f,%.4f,%i,%ld",
                            data->hg_ax,data->hg_ay,data->hg_az,
                            data->rocketState,data->timeStamp);
            break;
        case GPS:
            snprintf(dataChar,sizeof(dataChar),"%.4f,%.4f,%.4f,%d,%i,%ld",
                            data->latitude,data->longitude,data->altitude,
                            data->posLock,data->rocketState,data->timeStamp);
            break;
    }
    Serial.println(7);
    return dataChar;


    // char *buffer_string;
    // asprintf(&buffer_string,"%4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f\n",data->ax,data->ay,data->az,data->gx,data->gy,data->gz,data->mx,data->my,data->mz,rocketState,data->timeStamp);
    // return buffer_string;
}
