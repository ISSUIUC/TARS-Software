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


char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, lowg_dataStruct_t* data, FSM_State rocketState);
void logData(File* dataFile, highg_dataStruct_t* data, FSM_State rocketState);
void logData(File* dataFile, gps_dataStruct_t* data, FSM_State rocketState);

char* formatString(lowg_dataStruct_t* data, FSM_State rocketState);

#endif
