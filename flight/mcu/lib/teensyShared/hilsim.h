/*
 * Author: Anshuk Chigullapalli
 */


#include "Arduino.h"
#include "ChRt.h"
#include "SparkFun_Qwiic_KX13X.h"
#include "MS5611.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "RH_RF95.h"
#include "SparkFunLSM9DS1.h"
#include "Parse.h"

void hilsim_data_parser();

class HILSIM {
    struct outputData {
        float xData;
        float yData;
        float zData;
    };
    void hilsim_parser();
    void lsm_readAccel(LSM9DS1* lsm);
    void lsm_readGyro(LSM9DS1* lsm);
    void lsm_readMag(LSM9DS1* lsm);
    outputData highg_getAccelData();
    bool gps_getPVT(SFE_UBLOX_GNSS* gps, uint16_t maxWait);
    int32_t gps_getLatitude(SFE_UBLOX_GNSS* gps, uint16_t maxWait);
    int32_t gps_getLongitude(SFE_UBLOX_GNSS* gps, uint16_t maxWait);
    int32_t gps_getAltitude(SFE_UBLOX_GNSS* gps, uint16_t maxWait);
    int barometer_read(MS5611* barometer, uint8_t bits);

};
