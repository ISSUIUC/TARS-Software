#include "teensyShared.h"

//converts raw pressure transducer measurement to float representing pressure in pascals.
float ptConversion(uint16_t rawData){
    //placeholder until we get calibration data to do conversion. TODO: implement conversion    
    float pPascals = rawData;
    return pPascals;
}