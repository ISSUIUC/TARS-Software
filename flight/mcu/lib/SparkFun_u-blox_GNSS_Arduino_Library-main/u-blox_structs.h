/*
        This is a library written for the u-blox ZED-F9P and NEO-M8P-2
        SparkFun sells these at its website: www.sparkfun.com
        Do you like this library? Help support SparkFun. Buy a board!
        https://www.sparkfun.com/products/16481
        https://www.sparkfun.com/products/15136
        https://www.sparkfun.com/products/15005
        https://www.sparkfun.com/products/15733
        https://www.sparkfun.com/products/15193
        https://www.sparkfun.com/products/15210

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
        v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020

        This library handles configuring and handling the responses
        from a u-blox GPS module. Works with most modules from u-blox including
        the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

        https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

        Development environment specifics:
        Arduino IDE 1.8.13

        SparkFun code, firmware, and software is released under the MIT
  License(http://opensource.org/licenses/MIT). The MIT License (MIT) Copyright
  (c) 2016 SparkFun Electronics Permission is hereby granted, free of charge, to
  any person obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to permit
  persons to whom the Software is furnished to do so, subject to the following
  conditions:

        The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef __u_blox_structs_h__
#define __u_blox_structs_h__

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

#ifndef DEF_NUM_SENS
#define DEF_NUM_SENS 7  // The maximum number of ESF sensors
#endif

// Additional flags and pointers that need to be stored with each message type
struct ubxAutomaticFlags {
    union {
        uint8_t all;
        struct {
            uint8_t automatic : 1;  // Will this message be delivered and parsed
                                    // "automatically" (without polling)
            uint8_t implicitUpdate : 1;  // Is the update triggered by accessing
                                         // stale data (=true) or by a call to
                                         // checkUblox (=false)
            uint8_t addToFileBuffer : 1;  // Should the raw UBX data be added to
                                          // the file buffer?
            uint8_t callbackCopyValid : 1;  // Is the copy of the data struct
                                            // used by the callback valid/fresh?
        } bits;
    } flags;
};

// UBX-NAV-POSECEF (0x01 0x01): Position solution in ECEF
const uint16_t UBX_NAV_POSECEF_LEN = 20;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
    int32_t ecefX;  // ECEF X coordinate: cm
    int32_t ecefY;  // ECEF Y coordinate: cm
    int32_t ecefZ;  // ECEF Z coordinate: cm
    uint32_t pAcc;  // Position Accuracy Estimate: cm
} UBX_NAV_POSECEF_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t ecefX : 1;
            uint32_t ecefY : 1;
            uint32_t ecefZ : 1;
            uint32_t pAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_POSECEF_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_POSECEF_data_t data;
    UBX_NAV_POSECEF_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_POSECEF_data_t);
    UBX_NAV_POSECEF_data_t *callbackData;
} UBX_NAV_POSECEF_t;

// UBX-NAV-POSLLH (0x01 0x02): Geodetic position solution
const uint16_t UBX_NAV_POSLLH_LEN = 28;

typedef struct {
    uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
    int32_t lon;     // Longitude: Degrees * 1e-7
    int32_t lat;     // Latitude: Degrees * 1e-7
    int32_t height;  // Height above ellipsoid: mm
    int32_t hMSL;    // Height above mean sea level: mm
    uint32_t hAcc;   // Horizontal Accuracy Estimate: mm
    uint32_t vAcc;   // Vertical Accuracy Estimate: mm
} UBX_NAV_POSLLH_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t lon : 1;
            uint32_t lat : 1;
            uint32_t height : 1;
            uint32_t hMSL : 1;
            uint32_t hAcc : 1;
            uint32_t vAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_POSLLH_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_POSLLH_data_t data;
    UBX_NAV_POSLLH_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_POSLLH_data_t);
    UBX_NAV_POSLLH_data_t *callbackData;
} UBX_NAV_POSLLH_t;

// UBX-NAV-STATUS (0x01 0x03): Receiver navigation status
const uint16_t UBX_NAV_STATUS_LEN = 16;

typedef struct {
    uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
    uint8_t gpsFix;  // GPSfix Type: 0x00 = no fix; 0x01 = dead reckoning only;
                     // 0x02 = 2D-fix; 0x03 = 3D-fix 0x04 = GPS + dead reckoning
                     // combined; 0x05 = Time only fix; 0x06..0xff = reserved
    union {
        uint8_t all;
        struct {
            uint8_t gpsFixOk : 1;  // 1 = position and velocity valid and within
                                   // DOP and ACC Masks.
            uint8_t diffSoln : 1;  // 1 = differential corrections were applied
            uint8_t wknSet : 1;    // 1 = Week Number valid (see Time Validity
                                   // section for details)
            uint8_t towSet : 1;    // 1 = Time of Week valid (see Time Validity
                                   // section for details)
        } bits;
    } flags;
    union {
        uint8_t all;
        struct {
            uint8_t diffCorr : 1;  // 1 = differential corrections available
            uint8_t carrSolnValid : 1;  // 1 = valid carrSoln
            uint8_t reserved : 4;
            uint8_t mapMatching : 2;  // map matching status: 00: none
                                      // 01: valid but not used, i.e. map
                                      // matching data was received, but was too
                                      // old 10: valid and used, map matching
                                      // data was applied 11: valid and used,
                                      // map matching data was applied.
        } bits;
    } fixStat;
    union {
        uint8_t all;
        struct {
            uint8_t psmState : 2;  // power save mode state
                                   // 0: ACQUISITION [or when psm disabled]
                                   // 1: TRACKING
                                   // 2: POWER OPTIMIZED TRACKING
                                   // 3: INACTIVE
            uint8_t reserved1 : 1;
            uint8_t spoofDetState : 2;  // Spoofing detection state
                                        // 0: Unknown or deactivated
                                        // 1: No spoofing indicated
                                        // 2: Spoofing indicated
                                        // 3: Multiple spoofing indications
            uint8_t reserved2 : 1;
            uint8_t carrSoln : 2;  // Carrier phase range solution status:
                                   // 0: no carrier phase range solution
                                   // 1: carrier phase range solution with
                                   // floating ambiguities 2: carrier phase
                                   // range solution with fixed ambiguities
        } bits;
    } flags2;
    uint32_t ttff;  // Time to first fix (millisecond time tag): ms
    uint32_t msss;  // Milliseconds since Startup / Reset: ms
} UBX_NAV_STATUS_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t gpsFix : 1;

            uint32_t gpsFixOk : 1;
            uint32_t diffSoln : 1;
            uint32_t wknSet : 1;
            uint32_t towSet : 1;

            uint32_t diffCorr : 1;
            uint32_t carrSolnValid : 1;
            uint32_t mapMatching : 1;

            uint32_t psmState : 1;
            uint32_t spoofDetState : 1;
            uint32_t carrSoln : 1;

            uint32_t ttff : 1;
            uint32_t msss : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_STATUS_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_STATUS_data_t data;
    UBX_NAV_STATUS_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_STATUS_data_t);
    UBX_NAV_STATUS_data_t *callbackData;
} UBX_NAV_STATUS_t;

// UBX-NAV-DOP (0x01 0x04): Dilution of precision
const uint16_t UBX_NAV_DOP_LEN = 18;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
    uint16_t gDOP;  // Geometric DOP: * 0.01
    uint16_t pDOP;  // Position DOP: * 0.01
    uint16_t tDOP;  // Time DOP: * 0.01
    uint16_t vDOP;  // Vertical DOP: * 0.01
    uint16_t hDOP;  // Horizontal DOP: * 0.01
    uint16_t nDOP;  // Northing DOP: * 0.01
    uint16_t eDOP;  // Easting DOP: * 0.01
} UBX_NAV_DOP_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t gDOP : 1;
            uint32_t pDOP : 1;
            uint32_t tDOP : 1;
            uint32_t vDOP : 1;
            uint32_t hDOP : 1;
            uint32_t nDOP : 1;
            uint32_t eDOP : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_DOP_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_DOP_data_t data;
    UBX_NAV_DOP_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_DOP_data_t);
    UBX_NAV_DOP_data_t *callbackData;
} UBX_NAV_DOP_t;

// UBX-NAV-ATT (0x01 0x05): Attitude solution
const uint16_t UBX_NAV_ATT_LEN = 32;

typedef struct {
    uint32_t iTOW;    // GPS time of week of the navigation epoch: ms
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved1[3];
    int32_t roll;         // Vehicle roll: Degrees * 1e-5
    int32_t pitch;        // Vehicle pitch: Degrees * 1e-5
    int32_t heading;      // Vehicle heading: Degrees * 1e-5
    uint32_t accRoll;     // Vehicle roll accuracy (if null, roll angle is not
                          // available): Degrees * 1e-5
    uint32_t accPitch;    // Vehicle pitch accuracy (if null, roll angle is not
                          // available): Degrees * 1e-5
    uint32_t accHeading;  // Vehicle heading accuracy (if null, roll angle is
                          // not available): Degrees * 1e-5
} UBX_NAV_ATT_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t version : 1;
            uint32_t roll : 1;
            uint32_t pitch : 1;
            uint32_t heading : 1;
            uint32_t accRoll : 1;
            uint32_t accPitch : 1;
            uint32_t accHeading : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_ATT_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_ATT_data_t data;
    UBX_NAV_ATT_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_ATT_data_t);
    UBX_NAV_ATT_data_t *callbackData;
} UBX_NAV_ATT_t;

// UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
const uint16_t UBX_NAV_PVT_LEN = 92;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
    uint16_t year;  // Year (UTC)
    uint8_t month;  // Month, range 1..12 (UTC)
    uint8_t day;    // Day of month, range 1..31 (UTC)
    uint8_t hour;   // Hour of day, range 0..23 (UTC)
    uint8_t min;    // Minute of hour, range 0..59 (UTC)
    uint8_t sec;    // Seconds of minute, range 0..60 (UTC)
    union {
        uint8_t all;
        struct {
            uint8_t validDate : 1;      // 1 = valid UTC Date
            uint8_t validTime : 1;      // 1 = valid UTC time of day
            uint8_t fullyResolved : 1;  // 1 = UTC time of day has been fully
                                        // resolved (no seconds uncertainty).
            uint8_t validMag : 1;       // 1 = valid magnetic declination
        } bits;
    } valid;
    uint32_t tAcc;    // Time accuracy estimate (UTC): ns
    int32_t nano;     // Fraction of second, range -1e9 .. 1e9 (UTC): ns
    uint8_t fixType;  // GNSSfix Type:
                      // 0: no fix
                      // 1: dead reckoning only
                      // 2: 2D-fix
                      // 3: 3D-fix
                      // 4: GNSS + dead reckoning combined
                      // 5: time only fix
    union {
        uint8_t all;
        struct {
            uint8_t gnssFixOK : 1;  // 1 = valid fix (i.e within DOP & accuracy
                                    // masks)
            uint8_t diffSoln : 1;   // 1 = differential corrections were applied
            uint8_t psmState : 3;
            uint8_t
                headVehValid : 1;  // 1 = heading of vehicle is valid, only set
                                   // if the receiver is in sensor fusion mode
            uint8_t carrSoln : 2;  // Carrier phase range solution status:
                                   // 0: no carrier phase range solution
                                   // 1: carrier phase range solution with
                                   // floating ambiguities 2: carrier phase
                                   // range solution with fixed ambiguities
        } bits;
    } flags;
    union {
        uint8_t all;
        struct {
            uint8_t reserved : 5;
            uint8_t
                confirmedAvai : 1;  // 1 = information about UTC Date and Time
                                    // of Day validity confirmation is available
            uint8_t
                confirmedDate : 1;  // 1 = UTC Date validity could be confirmed
            uint8_t
                confirmedTime : 1;  // 1 = UTC Time of Day could be confirmed
        } bits;
    } flags2;
    uint8_t numSV;     // Number of satellites used in Nav Solution
    int32_t lon;       // Longitude: deg * 1e-7
    int32_t lat;       // Latitude: deg * 1e-7
    int32_t height;    // Height above ellipsoid: mm
    int32_t hMSL;      // Height above mean sea level: mm
    uint32_t hAcc;     // Horizontal accuracy estimate: mm
    uint32_t vAcc;     // Vertical accuracy estimate: mm
    int32_t velN;      // NED north velocity: mm/s
    int32_t velE;      // NED east velocity: mm/s
    int32_t velD;      // NED down velocity: mm/s
    int32_t gSpeed;    // Ground Speed (2-D): mm/s
    int32_t headMot;   // Heading of motion (2-D): deg * 1e-5
    uint32_t sAcc;     // Speed accuracy estimate: mm/s
    uint32_t headAcc;  // Heading accuracy estimate (both motion and vehicle):
                       // deg * 1e-5
    uint16_t pDOP;     // Position DOP * 0.01
    union {
        uint8_t all;
        struct {
            uint8_t invalidLlh : 1;  // 1 = Invalid lon, lat, height and hMSL
        } bits;
    } flags3;
    uint8_t reserved1[5];
    int32_t headVeh;  // Heading of vehicle (2-D): deg * 1e-5
    int16_t magDec;   // Magnetic declination: deg * 1e-2
    uint16_t magAcc;  // Magnetic declination accuracy: deg * 1e-2
} UBX_NAV_PVT_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t year : 1;
            uint32_t month : 1;
            uint32_t day : 1;
            uint32_t hour : 1;
            uint32_t min : 1;
            uint32_t sec : 1;

            uint32_t validDate : 1;
            uint32_t validTime : 1;
            uint32_t fullyResolved : 1;
            uint32_t validMag : 1;

            uint32_t tAcc : 1;
            uint32_t nano : 1;
            uint32_t fixType : 1;
            uint32_t gnssFixOK : 1;
            uint32_t diffSoln : 1;
            uint32_t psmState : 1;
            uint32_t headVehValid : 1;
            uint32_t carrSoln : 1;

            uint32_t confirmedAvai : 1;
            uint32_t confirmedDate : 1;
            uint32_t confirmedTime : 1;

            uint32_t numSV : 1;
            uint32_t lon : 1;
            uint32_t lat : 1;
            uint32_t height : 1;
            uint32_t hMSL : 1;
            uint32_t hAcc : 1;
            uint32_t vAcc : 1;
            uint32_t velN : 1;
            uint32_t velE : 1;
        } bits;
    } moduleQueried1;
    union {
        uint32_t all;
        struct {
            uint32_t velD : 1;
            uint32_t gSpeed : 1;
            uint32_t headMot : 1;
            uint32_t sAcc : 1;
            uint32_t headAcc : 1;
            uint32_t pDOP : 1;

            uint32_t invalidLlh : 1;

            uint32_t headVeh : 1;
            uint32_t magDec : 1;
            uint32_t magAcc : 1;
        } bits;
    } moduleQueried2;
} UBX_NAV_PVT_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_PVT_data_t data;
    UBX_NAV_PVT_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_PVT_data_t);
    UBX_NAV_PVT_data_t *callbackData;
} UBX_NAV_PVT_t;

// UBX-NAV-ODO (0x01 0x09): Odometer solution
const uint16_t UBX_NAV_ODO_LEN = 20;

typedef struct {
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved1[3];
    uint32_t iTOW;           // GPS time of week of the navigation epoch: ms
    uint32_t distance;       // Ground distance since last reset: m
    uint32_t totalDistance;  // Total cumulative ground distance: m
    uint32_t distanceStd;    // Ground distance accuracy (1-sigma): m
} UBX_NAV_ODO_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;
            uint32_t iTOW : 1;
            uint32_t distance : 1;
            uint32_t totalDistance : 1;
            uint32_t distanceStd : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_ODO_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_ODO_data_t data;
    UBX_NAV_ODO_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_ODO_data_t);
    UBX_NAV_ODO_data_t *callbackData;
} UBX_NAV_ODO_t;

// UBX-NAV-VELECEF (0x01 0x11): Velocity solution in ECEF
const uint16_t UBX_NAV_VELECEF_LEN = 20;

typedef struct {
    uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
    int32_t ecefVX;  // ECEF X velocity: cm/s
    int32_t ecefVY;  // ECEF Y velocity: cm/s
    int32_t ecefVZ;  // ECEF Z velocity: cm/s
    uint32_t sAcc;   // Speed accuracy estimate: cm/s
} UBX_NAV_VELECEF_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t ecefVX : 1;
            uint32_t ecefVY : 1;
            uint32_t ecefVZ : 1;
            uint32_t sAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_VELECEF_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_VELECEF_data_t data;
    UBX_NAV_VELECEF_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_VELECEF_data_t);
    UBX_NAV_VELECEF_data_t *callbackData;
} UBX_NAV_VELECEF_t;

// UBX-NAV-VELNED (0x01 0x12): Velocity solution in NED frame
const uint16_t UBX_NAV_VELNED_LEN = 36;

typedef struct {
    uint32_t iTOW;    // GPS time of week of the navigation epoch: ms
    int32_t velN;     // North velocity component: cm/s
    int32_t velE;     // East velocity component: cm/s
    int32_t velD;     // Down velocity component: cm/s
    uint32_t speed;   // Speed (3-D): cm/s
    uint32_t gSpeed;  // Ground Speed (2-D): cm/s
    int32_t heading;  // Heading of motion 2-D: Degrees * 1e-5
    uint32_t sAcc;    // Speed accuracy estimate: cm/s
    uint32_t cAcc;    // Course/Heading accuracy estimate: Degrees * 1e-5
} UBX_NAV_VELNED_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t velN : 1;
            uint32_t velE : 1;
            uint32_t velD : 1;
            uint32_t speed : 1;
            uint32_t gSpeed : 1;
            uint32_t heading : 1;
            uint32_t sAcc : 1;
            uint32_t cAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_VELNED_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_VELNED_data_t data;
    UBX_NAV_VELNED_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_VELNED_data_t);
    UBX_NAV_VELNED_data_t *callbackData;
} UBX_NAV_VELNED_t;

// UBX-NAV-HPPOSECEF (0x01 0x13): High precision position solution in ECEF
const uint16_t UBX_NAV_HPPOSECEF_LEN = 28;

typedef struct {
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved1[3];
    uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
    int32_t ecefX;   // ECEF X coordinate: cm
    int32_t ecefY;   // ECEF Y coordinate: cm
    int32_t ecefZ;   // ECEF Z coordinate: cm
    int8_t ecefXHp;  // High precision component of ECEF X coordinate: mm * 0.1
    int8_t ecefYHp;  // High precision component of ECEF Y coordinate: mm * 0.1
    int8_t ecefZHp;  // High precision component of ECEF Z coordinate: mm * 0.1
    union {
        uint8_t all;
        struct {
            uint8_t invalidEcef : 1;  // 1 = Invalid ecefX, ecefY, ecefZ,
                                      // ecefXHp, ecefYHp and ecefZHp
        } bits;
    } flags;
    uint32_t pAcc;  // Position Accuracy Estimate: mm * 0.1
} UBX_NAV_HPPOSECEF_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;
            uint32_t iTOW : 1;
            uint32_t ecefX : 1;
            uint32_t ecefY : 1;
            uint32_t ecefZ : 1;
            uint32_t ecefXHp : 1;
            uint32_t ecefYHp : 1;
            uint32_t ecefZHp : 1;

            uint32_t invalidEcef : 1;

            uint32_t pAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_HPPOSECEF_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_HPPOSECEF_data_t data;
    UBX_NAV_HPPOSECEF_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_HPPOSECEF_data_t);
    UBX_NAV_HPPOSECEF_data_t *callbackData;
} UBX_NAV_HPPOSECEF_t;

// UBX-NAV-HPPOSLLH (0x01 0x14): High precision geodetic position solution
const uint16_t UBX_NAV_HPPOSLLH_LEN = 36;

typedef struct {
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved1[2];
    union {
        uint8_t all;
        struct {
            uint8_t invalidLlh : 1;  // 1 = Invalid lon, lat, height, hMSL,
                                     // lonHp, latHp, heightHp and hMSLHp
        } bits;
    } flags;
    uint32_t iTOW;    // GPS time of week of the navigation epoch: ms
    int32_t lon;      // Longitude: deg * 1e-7
    int32_t lat;      // Latitude: deg * 1e-7
    int32_t height;   // Height above ellipsoid: mm
    int32_t hMSL;     // Height above mean sea level: mm
    int8_t lonHp;     // High precision component of longitude: deg * 1e-9
    int8_t latHp;     // High precision component of latitude: deg * 1e-9
    int8_t heightHp;  // High precision component of height above ellipsoid: mm
                      // * 0.1
    int8_t hMSLHp;  // High precision component of height above mean sea level:
                    // mm * 0.1
    uint32_t hAcc;  // Horizontal accuracy estimate: mm * 0.1
    uint32_t vAcc;  // Vertical accuracy estimate: mm * 0.1
} UBX_NAV_HPPOSLLH_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;

            uint32_t invalidLlh : 1;

            uint32_t iTOW : 1;
            uint32_t lon : 1;
            uint32_t lat : 1;
            uint32_t height : 1;
            uint32_t hMSL : 1;
            uint32_t lonHp : 1;
            uint32_t latHp : 1;
            uint32_t heightHp : 1;
            uint32_t hMSLHp : 1;
            uint32_t hAcc : 1;
            uint32_t vAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_HPPOSLLH_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_HPPOSLLH_data_t data;
    UBX_NAV_HPPOSLLH_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_HPPOSLLH_data_t);
    UBX_NAV_HPPOSLLH_data_t *callbackData;
} UBX_NAV_HPPOSLLH_t;

// UBX-NAV-TIMEUTC (0x01 0x21): UTC time solution
const uint16_t UBX_NAV_TIMEUTC_LEN = 20;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
    uint32_t tAcc;  // Time accuracy estimate (UTC): ns
    int32_t nano;   // Fraction of second, range -1e9 .. 1e9 (UTC): ns
    uint16_t year;  // Year (UTC)
    uint8_t month;  // Month, range 1..12 (UTC)
    uint8_t day;    // Day of month, range 1..31 (UTC)
    uint8_t hour;   // Hour of day, range 0..23 (UTC)
    uint8_t min;    // Minute of hour, range 0..59 (UTC)
    uint8_t sec;    // Seconds of minute, range 0..60 (UTC)
    union {
        uint8_t all;
        struct {
            uint8_t validTOW : 1;  // 1 = Valid Time of Week
            uint8_t validWKN : 1;  // 1 = Valid Week Number
            uint8_t validUTC : 1;  // 1 = Valid UTC Time
            uint8_t reserved : 1;
            uint8_t utcStandard : 4;  // UTC standard identifier
        } bits;
    } valid;
} UBX_NAV_TIMEUTC_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t tAcc : 1;
            uint32_t nano : 1;
            uint32_t year : 1;
            uint32_t month : 1;
            uint32_t day : 1;
            uint32_t hour : 1;
            uint32_t min : 1;
            uint32_t sec : 1;

            uint32_t validTOW : 1;
            uint32_t validWKN : 1;
            uint32_t validUTC : 1;
            uint32_t utcStandard : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_TIMEUTC_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_TIMEUTC_data_t data;
    UBX_NAV_TIMEUTC_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_TIMEUTC_data_t);
    UBX_NAV_TIMEUTC_data_t *callbackData;
} UBX_NAV_TIMEUTC_t;

// UBX-NAV-CLOCK (0x01 0x22): Clock solution
const uint16_t UBX_NAV_CLOCK_LEN = 20;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
    int32_t clkB;   // Clock bias: ns
    int32_t clkD;   // Clock drift: ns/s
    uint32_t tAcc;  // Time accuracy estimate: ns
    uint32_t fAcc;  // Frequency accuracy estimate: ps/s
} UBX_NAV_CLOCK_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t clkB : 1;
            uint32_t clkD : 1;
            uint32_t tAcc : 1;
            uint32_t fAcc : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_CLOCK_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_CLOCK_data_t data;
    UBX_NAV_CLOCK_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_CLOCK_data_t);
    UBX_NAV_CLOCK_data_t *callbackData;
} UBX_NAV_CLOCK_t;

// UBX-NAV-TIMELS (0x01 0x26): Leap second event information
const uint16_t UBX_NAV_TIMELS_LEN = 24;

typedef struct {
    uint32_t iTOW;    // GPS time of week of the navigation epoch: ms
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved1[3];
    uint8_t srcOfCurrLs;  // Information source for the current number of leap
                          // seconds
    int8_t currLs;  // Current number of leap seconds since start of GPS (Jan 6,
                    // 1980), s
    uint8_t
        srcOfLsChange;  // Information source for the future leap second event
    int8_t
        lsChange;  // Future leap second change if one is scheduled, +1, 0, -1s
    int32_t timeToLsEvent;   // Num of secs until the next or from the last leap
                             // second, s
    uint16_t dateOfLsGpsWn;  // GPS week num (WN) of the next or the last leap
                             // second event
    uint16_t dateOfLsGpsDn;  // GPS day of week num (DN) for the next or last
                             // leap second event
    uint8_t reserved2[3];
    union {
        uint8_t all;
        struct {
            uint8_t validCurrLs : 1;  // 1 = Valid current number of leap
                                      // seconds value
            uint8_t
                validTimeToLsEvent : 1;  // 1 = Valid time to next leap second
                                         // event or from the last leap second
                                         // event if no future event scheduled
        } bits;
    } valid;
} UBX_NAV_TIMELS_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t version : 1;
            uint32_t srcOfCurrLs : 1;
            uint32_t currLs : 1;
            uint32_t srcOfLsChange : 1;
            uint32_t lsChange : 1;
            uint32_t timeToLsEvent : 1;
            uint32_t dateOfLsGpsWn : 1;
            uint32_t dateOfLsGpsDn : 1;
            uint32_t validCurrLs : 1;
            uint32_t validTimeToLsEvent : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_TIMELS_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_TIMELS_data_t data;
    UBX_NAV_TIMELS_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_TIMELS_data_t);
    UBX_NAV_TIMELS_data_t *callbackData;
} UBX_NAV_TIMELS_t;

// UBX-NAV-SVIN (0x01 0x3B): Survey-in data
const uint16_t UBX_NAV_SVIN_LEN = 40;

typedef struct {
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved1[3];
    uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
    uint32_t dur;    // Passed survey-in observation time: s
    int32_t meanX;   // Current survey-in mean position ECEF X coordinate: cm
    int32_t meanY;   // Current survey-in mean position ECEF Y coordinate: cm
    int32_t meanZ;   // Current survey-in mean position ECEF Z coordinate: cm
    int8_t meanXHP;  // Current high-precision survey-in mean position ECEF X
                     // coordinate: mm * 0.1
    int8_t meanYHP;  // Current high-precision survey-in mean position ECEF Y
                     // coordinate: mm * 0.1
    int8_t meanZHP;  // Current high-precision survey-in mean position ECEF Z
                     // coordinate: mm * 0.1
    uint8_t reserved2;
    uint32_t meanAcc;  // Current survey-in mean position accuracy: mm * 0.1
    uint32_t obs;      // Number of position observations used during survey-in
    int8_t valid;   // Survey-in position validity flag, 1 = valid, otherwise 0
    int8_t active;  // Survey-in in progress flag, 1 = in-progress, otherwise 0
    uint8_t reserved3[2];
} UBX_NAV_SVIN_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;
            uint32_t iTOW : 1;
            uint32_t dur : 1;
            uint32_t meanX : 1;
            uint32_t meanY : 1;
            uint32_t meanZ : 1;
            uint32_t meanXHP : 1;
            uint32_t meanYHP : 1;
            uint32_t meanZHP : 1;
            uint32_t meanAcc : 1;
            uint32_t obs : 1;
            uint32_t valid : 1;
            uint32_t active : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_SVIN_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_SVIN_data_t data;
    UBX_NAV_SVIN_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_SVIN_data_t);
    UBX_NAV_SVIN_data_t *callbackData;
} UBX_NAV_SVIN_t;

// UBX-NAV-RELPOSNED (0x01 0x3C): Relative positioning information in NED frame
// Note:
//  RELPOSNED on the M8 is only 40 bytes long
//  RELPOSNED on the F9 is 64 bytes long and contains much more information
const uint16_t UBX_NAV_RELPOSNED_LEN = 40;
const uint16_t UBX_NAV_RELPOSNED_LEN_F9 = 64;

typedef struct {
    uint8_t version;  // Message version (0x00 for this version)
    uint8_t reserved0;
    uint16_t refStationId;  // Reference Station ID
    uint32_t iTOW;          // GPS time of week of the navigation epoch: ms
    int32_t relPosN;        // North component of relative position vector: cm
    int32_t relPosE;        // East component of relative position vector: cm
    int32_t relPosD;        // Down component of relative position vector: cm
    int32_t relPosLength;   // Length of the relative position vector: cm
    int32_t relPosHeading;  // Heading of the relative position vector: Degrees
                            // * 1e-5
    uint8_t reserved1[4];
    int8_t relPosHPN;  // High-precision North component of relative position
                       // vector: mm * 0.1
    int8_t relPosHPE;  // High-precision East component of relative position
                       // vector: mm * 0.1
    int8_t relPosHPD;  // High-precision Down component of relative position
                       // vector: mm * 0.1
    int8_t relPosHPLength;  // High-precision component of the length of the
                            // relative position vector: mm * 0.1
    uint32_t accN;  // Accuracy of relative position North component: mm * 0.1
    uint32_t accE;  // Accuracy of relative position East component: mm * 0.1
    uint32_t accD;  // Accuracy of relative position Down component: mm * 0.1
    uint32_t accLength;   // Accuracy of length of the relative position vector:
                          // mm * 0.1
    uint32_t accHeading;  // Accuracy of heading of the relative position
                          // vector: Degrees * 1e-5
    uint8_t reserved2[4];
    union {
        uint32_t all;
        struct {
            uint32_t
                gnssFixOK : 1;  // A valid fix (i.e within DOP & accuracy masks)
            uint32_t
                diffSoln : 1;  // 1 if differential corrections were applied
            uint32_t relPosValid : 1;  // 1 if relative position components and
                                       // accuracies are valid
            uint32_t carrSoln : 2;     // Carrier phase range solution status:
                                       // 0 = no carrier phase range solution
                                       // 1 = carrier phase range solution with
                                       // floating ambiguities 2 = carrier phase
                                       // range solution with fixed ambiguities
            uint32_t isMoving : 1;  // 1 if the receiver is operating in moving
                                    // baseline mode
            uint32_t refPosMiss : 1;  // 1 if extrapolated reference position
                                      // was used to compute moving baseline
                                      // solution this epoch
            uint32_t refObsMiss : 1;  // 1 if extrapolated reference
                                      // observations were used to compute
                                      // moving baseline solution this epoch
            uint32_t relPosHeadingValid : 1;  // 1 if relPosHeading is valid
            uint32_t
                relPosNormalized : 1;  // 1 if the components of the relative
                                       // position vector (including the
                                       // high-precision parts) are normalized
        } bits;
    } flags;
} UBX_NAV_RELPOSNED_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;
            uint32_t refStationId : 1;
            uint32_t iTOW : 1;
            uint32_t relPosN : 1;
            uint32_t relPosE : 1;
            uint32_t relPosD : 1;
            uint32_t relPosLength : 1;
            uint32_t relPosHeading : 1;
            uint32_t relPosHPN : 1;
            uint32_t relPosHPE : 1;
            uint32_t relPosHPD : 1;
            uint32_t relPosHPLength : 1;
            uint32_t accN : 1;
            uint32_t accE : 1;
            uint32_t accD : 1;
            uint32_t accLength : 1;
            uint32_t accHeading : 1;

            uint32_t gnssFixOK : 1;
            uint32_t diffSoln : 1;
            uint32_t relPosValid : 1;
            uint32_t carrSoln : 1;
            uint32_t isMoving : 1;
            uint32_t refPosMiss : 1;
            uint32_t refObsMiss : 1;
            uint32_t relPosHeadingValid : 1;
            uint32_t relPosNormalized : 1;
        } bits;
    } moduleQueried;
} UBX_NAV_RELPOSNED_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_NAV_RELPOSNED_data_t data;
    UBX_NAV_RELPOSNED_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_NAV_RELPOSNED_data_t);
    UBX_NAV_RELPOSNED_data_t *callbackData;
} UBX_NAV_RELPOSNED_t;

// RXM-specific structs

// UBX-RXM-SFRBX (0x02 0x13): Broadcast navigation data subframe
// Note: length is variable
// Note: on protocol version 17: numWords is (0..16)
//       on protocol version 18+: numWords is (0..10)
const uint8_t UBX_RXM_SFRBX_MAX_WORDS = 16;
const uint16_t UBX_RXM_SFRBX_MAX_LEN = 8 + (4 * UBX_RXM_SFRBX_MAX_WORDS);

typedef struct {
    uint8_t gnssId;  // GNSS identifier
    uint8_t svId;    // Satellite identifier
    uint8_t reserved1;
    uint8_t freqId;  // GLONASS frequency slot
    uint8_t
        numWords;  // The number of data words contained in this message (0..16)
    uint8_t chn;   // The tracking channel number the message was received on
    uint8_t version;  // Message version (0x01 for this version)
    uint8_t reserved2;
    uint32_t dwrd[UBX_RXM_SFRBX_MAX_WORDS];  // The data words
} UBX_RXM_SFRBX_data_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_RXM_SFRBX_data_t data;
    boolean moduleQueried;
    void (*callbackPointer)(UBX_RXM_SFRBX_data_t);
    UBX_RXM_SFRBX_data_t *callbackData;
} UBX_RXM_SFRBX_t;

// UBX-RXM-RAWX (0x02 0x15): Multi-GNSS raw measurement data
// Note: length is variable
const uint8_t UBX_RXM_RAWX_MAX_BLOCKS =
    92;  // See issue #70 and PR #74 for more info
const uint16_t UBX_RXM_RAWX_MAX_LEN = 16 + (32 * UBX_RXM_RAWX_MAX_BLOCKS);

typedef struct {
    uint8_t rcvTow[8];  // Measurement time of week in receiver local time
                        // [64-bit float]
    uint16_t week;      // GPS week number
    int8_t leapS;       // GPS leap seconds
    uint8_t numMeas;    // Number of measurements to follow
    union {
        uint8_t all;
        struct {
            uint8_t leapSec : 1;   // Leap seconds have been determined
            uint8_t clkReset : 1;  // Clock reset applied
        } bits;
    } recStat;
    uint8_t version;  // Message version (0x01 for this version)
    uint8_t reserved1[2];
} UBX_RXM_RAWX_header_t;

typedef struct {
    uint8_t prMes[8];   // Pseudorange measurement: m [64-bit float]
    uint8_t cpMes[8];   // Carrier phase measurement: cycles [64-bit float]
    uint8_t doMes[4];   // Doppler measurement: Hz [32-bit float]
    uint8_t gnssId;     // GNSS identifier
    uint8_t svId;       // Satellite identifier
    uint8_t sigId;      // New signal identifier
    uint8_t freqId;     // GLONASS frequency slot
    uint16_t lockTime;  // Carrier phase locktime counter: ms
    uint8_t cno;        // Carrier-to-noise density ratio: dB-Hz
    uint8_t prStdev;  // Estimated pseudorange measurement standard deviation: m
                      // * 0.01 * 2^n [4-bit]
    uint8_t cpStdev;  // Estimated carrier phase measurement standard deviation:
                      // cycles * 0.004 [4-bit]
    uint8_t doStdev;  // Estimated Doppler measurement standard deviation: Hz *
                      // 0.002 * 2^n [4-bit]
    union {
        uint8_t all;
        struct {
            uint8_t prValid : 1;     // Pseudorange valid
            uint8_t cpValid : 1;     // Carrier phase valid
            uint8_t halfCyc : 1;     // Half cycle valid
            uint8_t subHalfCyc : 1;  // Half cycle subtracted from phase
        } bits;
    } trkStat;
    uint8_t reserved2;
} UBX_RXM_RAWX_block_t;

typedef struct {
    UBX_RXM_RAWX_header_t header;
    UBX_RXM_RAWX_block_t blocks[UBX_RXM_RAWX_MAX_BLOCKS];
} UBX_RXM_RAWX_data_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_RXM_RAWX_data_t data;
    boolean moduleQueried;
    void (*callbackPointer)(UBX_RXM_RAWX_data_t);
    UBX_RXM_RAWX_data_t *callbackData;
} UBX_RXM_RAWX_t;

// CFG-specific structs

// UBX-CFG-RATE (0x06 0x08): Navigation/measurement rate settings
const uint16_t UBX_CFG_RATE_LEN = 6;

typedef struct {
    uint16_t measRate;  // The elapsed time between GNSS measurements, which
                        // defines the rate: ms
    uint16_t navRate;   // The ratio between the number of measurements and the
                        // number of navigation solutions: cycles
    uint16_t timeRef;   // The time system to which measurements are aligned: 0:
                        // UTC; 1: GPS; 2: GLONASS; 3: BeiDou; 4: Galileo
} UBX_CFG_RATE_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t measRate : 1;
            uint32_t navRate : 1;
            uint32_t timeRef : 1;
        } bits;
    } moduleQueried;
} UBX_CFG_RATE_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_CFG_RATE_data_t data;
    UBX_CFG_RATE_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_CFG_RATE_data_t);
    UBX_CFG_RATE_data_t *callbackData;
} UBX_CFG_RATE_t;

// TIM-specific structs

// UBX-TIM-TM2 (0x0D 0x03): Time mark data
const uint16_t UBX_TIM_TM2_LEN = 28;

typedef struct {
    uint8_t ch;  // Channel (i.e. EXTINT) upon which the pulse was measured
    union {
        uint8_t all;
        struct {
            uint8_t mode : 1;            // 0=single; 1=running
            uint8_t run : 1;             // 0=armed; 1=stopped
            uint8_t newFallingEdge : 1;  // New falling edge detected
            uint8_t timeBase : 2;  // 0=Time base is Receiver time; 1=Time base
                                   // is GNSS time; 2=Time base is UTC
            uint8_t utc : 1;       // 0=UTC not available; 1=UTC available
            uint8_t time : 1;  // 0=Time is not valid; 1=Time is valid (Valid
                               // GNSS fix)
            uint8_t newRisingEdge : 1;  // New rising edge detected
        } bits;
    } flags;
    uint16_t count;      // Rising edge counter
    uint16_t wnR;        // Week number of last rising edge
    uint16_t wnF;        // Week number of last falling edge
    uint32_t towMsR;     // TOW of rising edge: ms
    uint32_t towSubMsR;  // Millisecond fraction of tow of rising edge: ns
    uint32_t towMsF;     // TOW of falling edge: ms
    uint32_t towSubMsF;  // Millisecond fraction of tow of falling edge: ns
    uint32_t accEst;     // Accuracy estimate: ns
} UBX_TIM_TM2_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t ch : 1;
            uint32_t mode : 1;
            uint32_t run : 1;
            uint32_t newFallingEdge : 1;
            uint32_t timeBase : 1;
            uint32_t utc : 1;
            uint32_t time : 1;
            uint32_t newRisingEdge : 1;

            uint32_t count : 1;
            uint32_t wnR : 1;
            uint32_t wnF : 1;
            uint32_t towMsR : 1;
            uint32_t towSubMsR : 1;
            uint32_t towMsF : 1;
            uint32_t towSubMsF : 1;
            uint32_t accEst : 1;
        } bits;
    } moduleQueried;
} UBX_TIM_TM2_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_TIM_TM2_data_t data;
    UBX_TIM_TM2_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_TIM_TM2_data_t);
    UBX_TIM_TM2_data_t *callbackData;
} UBX_TIM_TM2_t;

// ESF-specific structs

// UBX-ESF-ALG (0x10 0x14): IMU alignment information
const uint16_t UBX_ESF_ALG_LEN = 16;

typedef struct {
    uint32_t iTOW;    // GPS time of week of the HNR epoch: ms
    uint8_t version;  // Message version (0x01 for this version)
    union {
        uint8_t all;
        struct {
            uint8_t
                autoMntAlgOn : 1;  // Automatic IMU-mount alignment on/off bit
            uint8_t status : 3;    // Status of the IMU-mount alignment
                                   //   0: user-defined/fixed angles are used
                                   //   1: IMU-mount roll/pitch angles alignment
                                   //   is ongoing 2: IMU-mount roll/pitch/yaw
                                   //   angles alignment is ongoing 3: coarse
                                   //   IMU-mount alignment are used 4: fine
                                   //   IMU-mount alignment are used
        } bits;
    } flags;
    union {
        uint8_t all;
        struct {
            uint8_t
                tiltAlgError : 1;     // IMU-mount tilt (roll and/or pitch)
                                      // alignment error (0: no error, 1: error)
            uint8_t yawAlgError : 1;  // IMU-mount yaw alignment error (0: no
                                      // error, 1: error)
            uint8_t
                angleError : 1;  // IMU-mount misalignment Euler angle
                                 // singularity error (0: no error, 1: error)
        } bits;
    } error;
    uint8_t reserved1;
    uint32_t yaw;   // IMU-mount yaw angle [0, 360]: Degrees * 1e-2
    int16_t pitch;  // IMU-mount pitch angle [-90, 90]: Degrees * 1e-2
    int16_t roll;   // IMU-mount roll angle [-180, 180]: Degrees * 1e-2
} UBX_ESF_ALG_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t version : 1;

            uint32_t autoMntAlgOn : 1;
            uint32_t status : 1;

            uint32_t tiltAlgError : 1;
            uint32_t yawAlgError : 1;
            uint32_t angleError : 1;

            uint32_t yaw : 1;
            uint32_t pitch : 1;
            uint32_t roll : 1;
        } bits;
    } moduleQueried;
} UBX_ESF_ALG_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_ESF_ALG_data_t data;
    UBX_ESF_ALG_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_ESF_ALG_data_t);
    UBX_ESF_ALG_data_t *callbackData;
} UBX_ESF_ALG_t;

// UBX-ESF-INS (0x10 0x15): Vehicle dynamics information
const uint16_t UBX_ESF_INS_LEN = 36;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t version : 8;  // Message version (0x01 for this version)
            uint32_t
                xAngRateValid : 1;  // Compensated x-axis angular rate data
                                    // validity flag (0: not valid, 1: valid)
            uint32_t
                yAngRateValid : 1;  // Compensated y-axis angular rate data
                                    // validity flag (0: not valid, 1: valid)
            uint32_t
                zAngRateValid : 1;     // Compensated z-axis angular rate data
                                       // validity flag (0: not valid, 1: valid)
            uint32_t xAccelValid : 1;  // Compensated x-axis acceleration data
                                       // validity flag (0: not valid, 1: valid)
            uint32_t yAccelValid : 1;  // Compensated y-axis acceleration data
                                       // validity flag (0: not valid, 1: valid)
            uint32_t zAccelValid : 1;  // Compensated z-axis acceleration data
                                       // validity flag (0: not valid, 1: valid)
        } bits;
    } bitfield0;
    uint8_t reserved1[4];
    uint32_t iTOW;     // GPS time of week of the HNR epoch: ms
    int32_t xAngRate;  // Compensated x-axis angular rate: Degrees/s * 1e-3
    int32_t yAngRate;  // Compensated y-axis angular rate: Degrees/s * 1e-3
    int32_t zAngRate;  // Compensated z-axis angular rate: Degrees/s * 1e-3
    int32_t
        xAccel;  // Compensated x-axis acceleration (gravity-free): m/s^2 * 1e-2
    int32_t
        yAccel;  // Compensated y-axis acceleration (gravity-free): m/s^2 * 1e-2
    int32_t
        zAccel;  // Compensated z-axis acceleration (gravity-free): m/s^2 * 1e-2
} UBX_ESF_INS_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;
            uint32_t xAngRateValid : 1;
            uint32_t yAngRateValid : 1;
            uint32_t zAngRateValid : 1;
            uint32_t xAccelValid : 1;
            uint32_t yAccelValid : 1;
            uint32_t zAccelValid : 1;

            uint32_t iTOW : 1;
            uint32_t xAngRate : 1;
            uint32_t yAngRate : 1;
            uint32_t zAngRate : 1;
            uint32_t xAccel : 1;
            uint32_t yAccel : 1;
            uint32_t zAccel : 1;
        } bits;
    } moduleQueried;
} UBX_ESF_INS_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_ESF_INS_data_t data;
    UBX_ESF_INS_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_ESF_INS_data_t);
    UBX_ESF_INS_data_t *callbackData;
} UBX_ESF_INS_t;

// UBX-ESF-MEAS (0x10 0x02): External sensor fusion measurements
// Note: length is variable
const uint16_t UBX_ESF_MEAS_MAX_LEN = 8 + (4 * DEF_NUM_SENS) + 4;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t dataField : 24;  // Data
            uint32_t
                dataType : 6;  // Type of data (0 = no data; 1..63 = data type)
        } bits;
    } data;
} UBX_ESF_MEAS_sensorData_t;

typedef struct {
    uint32_t timeTag;  // Time tag of measurement generated by external sensor
    union {
        uint16_t all;
        struct {
            uint16_t timeMarkSent : 2;  // Time mark signal was supplied just
                                        // prior to sending this message:
                                        //   0 = none, 1 = on Ext0, 2 = on Ext1
            uint16_t timeMarkEdge : 1;  // Trigger on rising (0) or falling (1)
                                        // edge of time mark signal
            uint16_t calibTtagValid : 1;  // Calibration time tag available.
                                          // Always set to zero.
            uint16_t reserved : 7;
            uint16_t numMeas : 5;  // Number of measurements contained in this
                                   // message (optional, can be obtained from
                                   // message size)
        } bits;
    } flags;
    uint16_t id;  // Identification number of data provider
    UBX_ESF_MEAS_sensorData_t data[DEF_NUM_SENS];
    uint32_t calibTtag;  // OPTIONAL: Receiver local time calibrated: ms
} UBX_ESF_MEAS_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t timeMarkSent : 1;
            uint32_t timeMarkEdge : 1;
            uint32_t calibTtagValid : 1;
            uint32_t numMeas : 1;

            uint32_t id : 1;

            uint32_t data : DEF_NUM_SENS;

            uint32_t calibTtag : 1;
        } bits;
    } moduleQueried;
} UBX_ESF_MEAS_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_ESF_MEAS_data_t data;
    UBX_ESF_MEAS_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_ESF_MEAS_data_t);
    UBX_ESF_MEAS_data_t *callbackData;
} UBX_ESF_MEAS_t;

// UBX-ESF-RAW (0x10 0x03): Raw sensor measurements
// Note: length is variable
const uint16_t UBX_ESF_RAW_MAX_LEN = 4 + (8 * DEF_NUM_SENS);

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t dataField : 24;  // Data
            uint32_t
                dataType : 8;  // Type of data (0 = no data; 1..255 = data type)
        } bits;
    } data;
    uint32_t sTag;  // Sensor time tag
} UBX_ESF_RAW_sensorData_t;

typedef struct {
    uint8_t reserved1[4];
    UBX_ESF_RAW_sensorData_t data[DEF_NUM_SENS];
} UBX_ESF_RAW_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t data : DEF_NUM_SENS;
        } bits;
    } moduleQueried;
} UBX_ESF_RAW_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_ESF_RAW_data_t data;
    UBX_ESF_RAW_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_ESF_RAW_data_t);
    UBX_ESF_RAW_data_t *callbackData;
} UBX_ESF_RAW_t;

// UBX-ESF-STATUS (0x10 0x10): External sensor fusion status
// Note: length is variable
const uint16_t UBX_ESF_STATUS_MAX_LEN = 16 + (4 * DEF_NUM_SENS);

typedef struct {
    union {
        uint8_t all;
        struct {
            uint8_t type : 6;   // Sensor data type
            uint8_t used : 1;   // If set, sensor data is used for the current
                                // sensor fusion solution
            uint8_t ready : 1;  // If set, sensor is set up (configuration is
                                // available or not required) but not used for
                                // computing the current sensor fusion solution.
        } bits;
    } sensStatus1;
    union {
        uint8_t all;
        struct {
            uint8_t calibStatus : 2;  // 00: Sensor is not calibrated
                                      // 01: Sensor is calibrating
                                      // 10/11: Sensor is calibrated
            uint8_t timeStatus : 2;   // 00: No data
                                      // 01: Reception of the first byte used to
            // tag the measurement 10: Event input used
            // to tag the measurement 11: Time tag
            // provided with the data
        } bits;
    } sensStatus2;
    uint8_t freq;  // Observation frequency: Hz
    union {
        uint8_t all;
        struct {
            uint8_t badMeas : 1;      // Bad measurements detected
            uint8_t badTTag : 1;      // Bad measurement time-tags detected
            uint8_t missingMeas : 1;  // Missing or time-misaligned measurements
                                      // detected
            uint8_t noisyMeas : 1;    // High measurement noise-level detected
        } bits;
    } faults;
} UBX_ESF_STATUS_sensorStatus_t;

typedef struct {
    uint32_t iTOW;    // GPS time of week of the HNR epoch: ms
    uint8_t version;  // Message version (0x02 for this version)
    uint8_t reserved1[7];
    uint8_t
        fusionMode;  // Fusion mode:
                     //  0: Initialization mode: receiver is initializing some
                     //  unknown values required for doing sensor fusion 1:
                     //  Fusion mode: GNSS and sensor data are used for
                     //  navigation solution computation 2: Suspended fusion
                     //  mode: sensor fusion is temporarily disabled due to e.g.
                     //  invalid sensor data or detected ferry 3: Disabled
                     //  fusion mode: sensor fusion is permanently disabled
                     //  until receiver reset due e.g. to sensor error
    uint8_t reserved2[2];
    uint8_t numSens;  // Number of sensors
    UBX_ESF_STATUS_sensorStatus_t status[DEF_NUM_SENS];
} UBX_ESF_STATUS_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t version : 1;
            uint32_t fusionMode : 1;
            uint32_t numSens : 1;

            uint32_t status : DEF_NUM_SENS;
        } bits;
    } moduleQueried;
} UBX_ESF_STATUS_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_ESF_STATUS_data_t data;
    UBX_ESF_STATUS_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_ESF_STATUS_data_t);
    UBX_ESF_STATUS_data_t *callbackData;
} UBX_ESF_STATUS_t;

// HNR-specific structs

// UBX-HNR-PVT (0x28 0x00): High rate output of PVT solution
const uint16_t UBX_HNR_PVT_LEN = 72;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the HNR epoch: ms
    uint16_t year;  // Year (UTC)
    uint8_t month;  // Month, range 1..12 (UTC)
    uint8_t day;    // Day of month, range 1..31 (UTC)
    uint8_t hour;   // Hour of day, range 0..23 (UTC)
    uint8_t min;    // Minute of hour, range 0..59 (UTC)
    uint8_t sec;    // Seconds of minute, range 0..60 (UTC)
    union {
        uint8_t all;
        struct {
            uint8_t validDate : 1;      // 1 = Valid UTC Date
            uint8_t validTime : 1;      // 1 = Valid UTC Time of Day
            uint8_t fullyResolved : 1;  // 1 = UTC Time of Day has been fully
                                        // resolved
        } bits;
    } valid;
    int32_t nano;    // Fraction of second (UTC): ns
    uint8_t gpsFix;  // GPSfix Type, range 0..5
                     // 0x00 = No Fix
                     // 0x01 = Dead Reckoning only
                     // 0x02 = 2D-Fix
                     // 0x03 = 3D-Fix
                     // 0x04 = GPS + dead reckoning combined
                     // 0x05 = Time only fix
                     // 0x06..0xff: reserved
    union {
        uint8_t all;
        struct {
            uint8_t
                gpsFixOK : 1;  // >1 = Fix within limits (e.g. DOP & accuracy)
            uint8_t diffSoln : 1;  // 1 = DGPS used
            uint8_t WKNSET : 1;    // 1 = Valid GPS week number
            uint8_t TOWSET : 1;    // 1 = Valid GPS time of week (iTOW & fTOW)
            uint8_t headVehValid : 1;  // 1= Heading of vehicle is valid
        } bits;
    } flags;
    uint8_t reserved1[2];
    int32_t lon;       // Longitude: Degrees * 1e-7
    int32_t lat;       // Latitude: Degrees * 1e-7
    int32_t height;    // Height above ellipsoid: mm
    int32_t hMSL;      // Height above MSL: mm
    int32_t gSpeed;    // Ground Speed (2-D): mm/s
    int32_t speed;     // Speed (3-D): mm/s
    int32_t headMot;   // Heading of motion (2-D): Degrees * 1e-5
    int32_t headVeh;   // Heading of vehicle (2-D): Degrees * 1e-5
    uint32_t hAcc;     // Horizontal accuracy: mm
    uint32_t vAcc;     // Vertical accuracy: mm
    uint32_t sAcc;     // Speed accuracy: mm/s
    uint32_t headAcc;  // Heading accuracy: Degrees * 1e-5
    uint8_t reserved2[4];
} UBX_HNR_PVT_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t year : 1;
            uint32_t month : 1;
            uint32_t day : 1;
            uint32_t hour : 1;
            uint32_t min : 1;
            uint32_t sec : 1;

            uint32_t validDate : 1;
            uint32_t validTime : 1;
            uint32_t fullyResolved : 1;

            uint32_t nano : 1;
            uint32_t gpsFix : 1;

            uint32_t gpsFixOK : 1;
            uint32_t diffSoln : 1;
            uint32_t WKNSET : 1;
            uint32_t TOWSET : 1;
            uint32_t headVehValid : 1;

            uint32_t lon : 1;
            uint32_t lat : 1;
            uint32_t height : 1;
            uint32_t hMSL : 1;
            uint32_t gSpeed : 1;
            uint32_t speed : 1;
            uint32_t headMot : 1;
            uint32_t headVeh : 1;
            uint32_t hAcc : 1;
            uint32_t vAcc : 1;
            uint32_t sAcc : 1;
            uint32_t headAcc : 1;
        } bits;
    } moduleQueried;
} UBX_HNR_PVT_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_HNR_PVT_data_t data;
    UBX_HNR_PVT_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_HNR_PVT_data_t);
    UBX_HNR_PVT_data_t *callbackData;
} UBX_HNR_PVT_t;

// UBX-HNR-ATT (0x28 0x01): Attitude solution
const uint16_t UBX_HNR_ATT_LEN = 32;

typedef struct {
    uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
    uint8_t version;
    uint8_t reserved1[3];
    int32_t roll;         // Vehicle roll: Degrees * 1e-5
    int32_t pitch;        // Vehicle pitch: Degrees * 1e-5
    int32_t heading;      // Vehicle heading: Degrees * 1e-5
    uint32_t accRoll;     // Vehicle roll accuracy: Degrees * 1e-5
    uint32_t accPitch;    // Vehicle pitch accuracy: Degrees * 1e-5
    uint32_t accHeading;  // Vehicle heading accuracy: Degrees * 1e-5
} UBX_HNR_ATT_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t iTOW : 1;
            uint32_t version : 1;
            uint32_t roll : 1;
            uint32_t pitch : 1;
            uint32_t heading : 1;
            uint32_t accRoll : 1;
            uint32_t accPitch : 1;
            uint32_t accHeading : 1;
        } bits;
    } moduleQueried;
} UBX_HNR_ATT_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_HNR_ATT_data_t data;
    UBX_HNR_ATT_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_HNR_ATT_data_t);
    UBX_HNR_ATT_data_t *callbackData;
} UBX_HNR_ATT_t;

// UBX-HNR-INS (0x28 0x02): Vehicle dynamics information
const uint16_t UBX_HNR_INS_LEN = 36;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t version : 8;  // Message version (0x00 for this version)
            uint32_t
                xAngRateValid : 1;  // Compensated x-axis angular rate data
                                    // validity flag (0: not valid, 1: valid)
            uint32_t
                yAngRateValid : 1;  // Compensated y-axis angular rate data
                                    // validity flag (0: not valid, 1: valid)
            uint32_t
                zAngRateValid : 1;     // Compensated z-axis angular rate data
                                       // validity flag (0: not valid, 1: valid)
            uint32_t xAccelValid : 1;  // Compensated x-axis acceleration data
                                       // validity flag (0: not valid, 1: valid)
            uint32_t yAccelValid : 1;  // Compensated y-axis acceleration data
                                       // validity flag (0: not valid, 1: valid)
            uint32_t zAccelValid : 1;  // Compensated z-axis acceleration data
                                       // validity flag (0: not valid, 1: valid)
        } bits;
    } bitfield0;
    uint8_t reserved1[4];
    uint32_t iTOW;     // GPS time of week of the HNR epoch: ms
    int32_t xAngRate;  // Compensated x-axis angular rate: Degrees/s * 1e-3
    int32_t yAngRate;  // Compensated y-axis angular rate: Degrees/s * 1e-3
    int32_t zAngRate;  // Compensated z-axis angular rate: Degrees/s * 1e-3
    int32_t
        xAccel;  // Compensated x-axis acceleration (with gravity): m/s^2 * 1e-2
    int32_t
        yAccel;  // Compensated y-axis acceleration (with gravity): m/s^2 * 1e-2
    int32_t
        zAccel;  // Compensated z-axis acceleration (with gravity): m/s^2 * 1e-2
} UBX_HNR_INS_data_t;

typedef struct {
    union {
        uint32_t all;
        struct {
            uint32_t all : 1;

            uint32_t version : 1;
            uint32_t xAngRateValid : 1;
            uint32_t yAngRateValid : 1;
            uint32_t zAngRateValid : 1;
            uint32_t xAccelValid : 1;
            uint32_t yAccelValid : 1;
            uint32_t zAccelValid : 1;

            uint32_t iTOW : 1;
            uint32_t xAngRate : 1;
            uint32_t yAngRate : 1;
            uint32_t zAngRate : 1;
            uint32_t xAccel : 1;
            uint32_t yAccel : 1;
            uint32_t zAccel : 1;
        } bits;
    } moduleQueried;
} UBX_HNR_INS_moduleQueried_t;

typedef struct {
    ubxAutomaticFlags automaticFlags;
    UBX_HNR_INS_data_t data;
    UBX_HNR_INS_moduleQueried_t moduleQueried;
    void (*callbackPointer)(UBX_HNR_INS_data_t);
    UBX_HNR_INS_data_t *callbackData;
} UBX_HNR_INS_t;

// UBX-CFG-TP5 (0x06 0x31): Time pulse parameters
const uint16_t UBX_CFG_TP5_LEN = 32;

typedef struct {
    uint8_t tpIdx;    // Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
    uint8_t version;  // Message version (0x01 for this version)
    uint8_t reserved1[2];
    int16_t antCableDelay;  // Antenna cable delay: ns
    int16_t rfGroupDelay;   // RF group delay: ns
    uint32_t freqPeriod;    // Frequency or period time, depending on setting of
                            // bit 'isFreq': Hz_or_us
    uint32_t
        freqPeriodLock;  // Frequency or period time when locked to GNSS time,
                         // only used if 'lockedOtherSet' is set: Hz_or_us
    uint32_t pulseLenRatio;      // Pulse length or duty cycle, depending on
                                 // 'isLength': us_or_2^-32
    uint32_t pulseLenRatioLock;  // Pulse length or duty cycle when locked to
                                 // GNSS time, only used if 'lockedOtherSet' is
                                 // set: us_or_2^-32
    int32_t userConfigDelay;     // User-configurable time pulse delay: ns
    union {
        uint32_t all;
        struct {
            uint32_t active : 1;  // If set enable time pulse; if pin assigned
                                  // to another function, other function takes
                                  // precedence.
            uint32_t lockGnssFreq : 1;    // If set, synchronize time pulse to
                                          // GNSS as soon as GNSS time is valid.
                                          // If not set, or before GNSS time is
                                          // valid, use local clock.
            uint32_t lockedOtherSet : 1;  // If set the receiver switches
                                          // between the timepulse settings
                                          // given by 'freqPeriodLocked' &
                                          // 'pulseLenLocked' and those given by
                                          // 'freqPeriod' & 'pulseLen'.
            uint32_t isFreq : 1;  // If set 'freqPeriodLock' and 'freqPeriod'
                                  // are interpreted as frequency, otherwise
                                  // interpreted as period.
            uint32_t
                isLength : 1;  // If set 'pulseLenRatioLock' and 'pulseLenRatio'
                               // interpreted as pulse length, otherwise
                               // interpreted as duty cycle.
            uint32_t
                alignToTow : 1;     // Align pulse to top of second (period time
                                    // must be integer fraction of 1s). Also set
                                    // 'lockGnssFreq' to use this feature.
            uint32_t polarity : 1;  // Pulse polarity: 0: falling edge at top of
                                    // second; 1: rising edge at top of second
            uint32_t gridUtcGnss : 4;  // Timegrid to use: 0: UTC; 1: GPS; 2:
                                       // GLONASS; 3: BeiDou; 4: Galileo
            uint32_t
                syncMode : 3;  // Sync Manager lock mode to use:
                               // 0: switch to 'freqPeriodLock' and
                               // 'pulseLenRatioLock' as soon as Sync Manager
                               // has an accurate time, never switch back to
                               // 'freqPeriod' and 'pulseLenRatio' 1: switch to
                               // 'freqPeriodLock' and 'pulseLenRatioLock' as
                               // soon as Sync Manager has an accurate time, and
                               // switch back to 'freqPeriod' and
                               // 'pulseLenRatio' as soon as time gets
                               // inaccurate
        } bits;
    } flags;
} UBX_CFG_TP5_data_t;

#endif
