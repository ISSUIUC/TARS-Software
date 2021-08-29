#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

#include <stdint.h>

// old version. Switch away from it later:
struct dataStruct_t {
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

    // GPS DATA
    float latitude;
    float longitude;
    float altitude;
    bool posLock;

    int32_t timeStamp;
};

// this is the new version. Switch to it later:
/*struct lowg_dataStruct_t {
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

};*/

#endif