#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <SD.h>
#include <ChRt.h>

#include "acShared.h"





struct sensorDataStruct_t {
    //! data for lowGimu
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;

    //! data for highGimu accel data (hg_x, hg_y, hg_z) 
    float hg_ax;
    float hg_ay;
    float hg_az;
    // int16_t pt1;
    // int16_t pt2;
    // int16_t pt3;

    //GPS DATA
    float latitude;
    float longitude;
    float altitude;
    bool posLock;
    
    // Data for all
    FSM_State rocketState;
    int32_t timeStamp;

};

enum sensors {
    LOWG_IMU,
    HIGHG_IMU,
    GPS
};


#define FIFO_SIZE 1000
struct datalogger_THD {
    //semaphore_t fifoData;
    SEMAPHORE_DECL(fifoData, 0);
    SEMAPHORE_DECL(fifoSpace, FIFO_SIZE);

    uint16_t fifoHead = 0;
    uint16_t fifoTail = 0;

    uint16_t bufferErrors = 0;

    MUTEX_DECL(dataMutex);

    sensorDataStruct_t fifoArray[FIFO_SIZE];

    sensorDataStruct_t current_data;

    sensors sensor_type;

    File dataFile;
};



char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, sensorDataStruct_t* data, sensors sensorType);

#endif
