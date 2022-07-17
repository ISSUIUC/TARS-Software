/**
 * @file dataLog.cpp
 *
 * Contains the code to handle FIFO buffers for sensors,
 * work with the SD card library. The header file also contains
 * important data structs.
 */

#ifndef DATALOG_CPP
#define DATALOG_CPP

#include "dataLog.h"

#include <ChRt.h>
#include <SD.h>
#include <stdio.h>

#include "pins.h"
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

        DataLogBuffer& buffers = pointer_struct->dataloggerTHDVarsPointer;

        // read each fifo once checking if they have data
        current_data.has_lowG_data =
            buffers.popLowGFifo(&current_data.lowG_data);

        current_data.has_highG_data =
            buffers.popHighGFifo(&current_data.highG_data);

        current_data.has_gps_data = buffers.popGpsFifo(&current_data.gps_data);

        current_data.has_state_data =
            buffers.popStateFifo(&current_data.state_data);

        current_data.has_rocketState_data =
            buffers.popRocketStateFifo(&current_data.rocketState_data);

        current_data.has_barometer_data =
            buffers.popBarometerFifo(&current_data.barometer_data);

        current_data.has_flap_data =
            buffers.popFlapsFifo(&current_data.flap_data);

        current_data.has_voltage_data =
            buffers.popVoltageFifo(&current_data.voltage_data);

        // check if any buffers have data
        bool any_have_data =
            current_data.has_gps_data || current_data.has_highG_data ||
            current_data.has_lowG_data || current_data.has_rocketState_data ||
            current_data.has_state_data || current_data.has_barometer_data ||
            current_data.has_flap_data || current_data.has_voltage_data;

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
    // Flush data once for every 50 writes
    // Flushing data is the step that actually writes to the card
    // Flushing more frequently incurs more of a latency penalty, but less
    // potential data loss
    if (flush_iterator == 50) {
        dataFile->flush();
        flush_iterator = 0;
    } else {
        flush_iterator++;
    }
}

bool DataLogBuffer::pushLowGFifo(LowGData* lowG_Data) {
    return lowGFifo.push(*lowG_Data);
}

bool DataLogBuffer::popLowGFifo(LowGData* lowG_Data) {
    return lowGFifo.pop(lowG_Data);
}

bool DataLogBuffer::pushHighGFifo(HighGData* highG_Data) {
    return highGFifo.push(*highG_Data);
}

bool DataLogBuffer::popHighGFifo(HighGData* highG_Data) {
    return highGFifo.pop(highG_Data);
}

bool DataLogBuffer::pushGpsFifo(GpsData* gps_Data) {
    return gpsFifo.push(*gps_Data);
}

bool DataLogBuffer::popGpsFifo(GpsData* gps_Data) {
    return gpsFifo.pop(gps_Data);
}

bool DataLogBuffer::pushStateFifo(stateData* state_data) {
    return stateFifo.push(*state_data);
}

bool DataLogBuffer::popStateFifo(stateData* state_data) {
    return stateFifo.pop(state_data);
}

bool DataLogBuffer::pushBarometerFifo(BarometerData* barometer_data) {
    return barometerFifo.push(*barometer_data);
}

bool DataLogBuffer::popBarometerFifo(BarometerData* barometer_data) {
    return barometerFifo.pop(barometer_data);
}

bool DataLogBuffer::pushRocketStateFifo(rocketStateData* rocket_data) {
    return rocketStateFifo.push(*rocket_data);
}

bool DataLogBuffer::popRocketStateFifo(rocketStateData* rocket_data) {
    return rocketStateFifo.pop(rocket_data);
}

bool DataLogBuffer::pushFlapsFifo(FlapData* flap_data) {
    return flapFifo.push(*flap_data);
}

bool DataLogBuffer::popFlapsFifo(FlapData* flap_data) {
    return flapFifo.pop(flap_data);
}

bool DataLogBuffer::pushVoltageFifo(VoltageData* voltage_data) {
    return voltageFifo.push(*voltage_data);
}

bool DataLogBuffer::popVoltageFifo(VoltageData* voltage_data) {
    return voltageFifo.pop(voltage_data);
}

#endif