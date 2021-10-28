#ifndef DATALOG_CPP
#define DATALOG_CPP

#include "dataLog.h"

#include <ChRt.h>
#include <SD.h>
#include <stdio.h>

#include "sensors.h"

MUTEX_DECL(SD_Card_Mutex);

/**
 * @brief Construct a new thd function object to log data to the SD card.
 *
 * @param arg Contains info on the ring buffers containing the data and mutexes
 * protecting it. This allows data to be passed to the function from another
 * file.
 *
 */
void dataLoggerTickFunction(pointers* pointer_struct) {
    // Initialize a new data struct object to hold the data that will be
    // logged

    // write to the sd card while any buffers have data
    while (true) {
        sensorDataStruct_t current_data{};

        datalogger_THD& buffers = pointer_struct->dataloggerTHDVarsPointer;

        // read each fifo once checking if they have data
        current_data.has_lowG_data =
            buffers.lowGFifo.pop(&current_data.lowG_data);

        current_data.has_highG_data =
            buffers.highGFifo.pop(&current_data.highG_data);

        current_data.has_gps_data = buffers.gpsFifo.pop(&current_data.gps_data);

        current_data.has_state_data =
            buffers.stateFifo.pop(&current_data.state_data);

        current_data.has_rocketState_data =
            buffers.rocketStateFifo.pop(&current_data.rocketState_data);

        current_data.has_barometer_data =
            buffers.barometerFifo.pop(&current_data.barometer_data);

        // check if any buffers have data
        bool any_have_data =
            current_data.has_gps_data || current_data.has_highG_data ||
            current_data.has_lowG_data || current_data.has_rocketState_data ||
            current_data.has_state_data || current_data.has_barometer_data;

        if (!any_have_data) {
            return;
        }

        // Log all data that was copied from the buffer onto the sd card
        chMtxLock(&SD_Card_Mutex);
        logData(&pointer_struct->dataloggerTHDVarsPointer.dataFile,
                &current_data);
        chMtxUnlock(&SD_Card_Mutex);
    }
}

/**
 * @brief Creates the name for a file to be written to SD card.
 *
 * @param fileName Pointer to char[] containing intended name of file. Do not
 * include number or file extension at end of name. Make sure this is longer
 * than it needs to be.
 * @param fileExtension Pointer to char[] containing the file extension for the
 * file.
 * @return char* Pointer to inputted char[]. It now contains number (if
 * duplicate file existed) and .csv file extension.
 */
char* sd_file_namer(char* fileName, char* fileExtensionParam) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[strlen(fileName) + 1];
    strcpy(inputName, fileName);

    strcat(fileName, fileExtension);

    // checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)) {
        bool fileExists = false;
        int i = 1;
        while (fileExists == false) {
            if (i > 999) {
                // max number of files reached. Don't want to overflow
                // fileName[]. Will write new data to already existing
                // data999.csv
                strcpy(fileName, inputName);
                strcat(fileName, "999");
                strcat(fileName, fileExtension);
                break;
            }

            // converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            // writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName) + strlen(iStr) + 6];
            strcpy(fileNameTemp, inputName);
            strcat(fileNameTemp, iStr);
            strcat(fileNameTemp, fileExtension);

            if (!SD.exists(fileNameTemp)) {
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
 */
int32_t flush_iterator = 0;
void logData(File* dataFile, sensorDataStruct_t* data) {
    // Write raw bytes to SD card.
    dataFile->write((const uint8_t*)data, sizeof(*data));

    // Flush data once for every 1000 writes (this keeps the ring buffer in sync
    // with data collection)
    if (flush_iterator == 1000) {
        dataFile->flush();
        flush_iterator = 0;
    } else {
        flush_iterator++;
    }
}

#endif
