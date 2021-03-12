#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <SD.h>
#include <ChRt.h>

#include "acShared.h"



struct lowg_dataStruct_t {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    
    int32_t timeStamp;

};

struct highg_dataStruct_t {
    //! data for highGimu accel data (hg_x, hg_y, hg_z) 
    float hg_ax;
    float hg_ay;
    float hg_az;
    // int16_t pt1;
    // int16_t pt2;
    // int16_t pt3;
    
    int32_t timeStamp;

};

struct gps_dataStruct_t {
    //GPS DATA
    float latitude;
    float longitude;
    float altitude;
    bool posLock;
    
    int32_t timeStamp;

};

#define FIFO_SIZE 3000
struct datalogger_THD {
    //semaphore_t fifoData;
    SEMAPHORE_DECL(fifoData, 0);
    SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

    uint16_t fifoHead = 0;
    uint16_t fifoTail = 0;

    uint16_t bufferErrors = 0;

    MUTEX_DECL(dataMutex);

    lowg_dataStruct_t fifoArray[FIFO_SIZE];

    lowg_dataStruct_t current_data;

    File dataFile;
};

// static THD_FUNCTION(dataLogger_THD,datalogger_THD arg);

char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, lowg_dataStruct_t* data);
void logData(File* dataFile, highg_dataStruct_t* data, FSM_State rocketState);
void logData(File* dataFile, gps_dataStruct_t* data, FSM_State rocketState);

char* formatString(lowg_dataStruct_t* data, FSM_State rocketState);

#endif
