#include <SD.h>

#include "dataLog.h"

#include <stdio.h>

#include <string>
#include <cstring>



static THD_FUNCTION(dataLogger_THD, arg){
  struct datalogger_THD *datalogger_struct = (struct datalogger_THD *)arg;
  while(true){
    #ifdef THREAD_DEBUG
      Serial.println("### Low g Data Logging thread entrance");
    #endif
    chSemWait(&datalogger_struct->fifoData);
    chMtxLock(&datalogger_struct->dataMutex);

    datalogger_struct->current_data = datalogger_struct->fifoArray[datalogger_struct->fifoTail];

    datalogger_struct->fifoTail = datalogger_struct->fifoTail < (FIFO_SIZE - 1) ? datalogger_struct->fifoTail + 1 : 0;
    chSemSignal(&datalogger_struct->fifoSpace);
    chMtxUnlock(&datalogger_struct->dataMutex);
    
    logData(&datalogger_struct->dataFile, &datalogger_struct->current_data);

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

    Serial.println(fileName);
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
void logData(File* dataFile, lowg_dataStruct_t* data) {

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
    
    dataFile->print(data->timeStamp);
    dataFile->print("\n");

    // For more efficient printing to file (WIP)
    /* char *buffer_string = formatString(data,rocketState);

    dataFile->print(buffer_string);

    free(buffer_string); */

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



/* char* formatString(lowg_dataStruct_t* data, FSM_State rocketState) {
    std::string buffer_string = std::to_string(data->ax);
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
    buffer_string.append(str(rocketState));
    buffer_string.append(", ");
    buffer_string.append(str(data->timeStamp));
    
    return buffer_string.c_str();


    // char *buffer_string;
    // asprintf(&buffer_string,"%4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f\n",data->ax,data->ay,data->az,data->gx,data->gy,data->gz,data->mx,data->my,data->mz,rocketState,data->timeStamp);
    // return buffer_string;
} */
