#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <SD.h>

#include "acShared.h"
#include "dataStructs.h"

void init_dataLog(File* dataFile);

void logData(File* dataFile, dataStruct_t* data, FSM_State rocketState);

#endif
