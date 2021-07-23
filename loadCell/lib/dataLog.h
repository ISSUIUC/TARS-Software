#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <SD.h>

#include "acShared.h"

struct dataStruct_t {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t mx;
    int16_t my;
    int16_t mz;
    // int16_t pt1;
    // int16_t pt2;
    // int16_t pt3;
    int32_t timeStamp;

};

void init_dataLog(File* dataFile);

void logData(File* dataFile, dataStruct_t* data, FSM_State rocketState);

#endif
