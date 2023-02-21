//
//    FILE: MS5611.cpp
//  AUTHOR: Rob Tillaart
//          Erni - testing/fixes
//          Roy van der Kraan - partial rewrite (mainly for SPI use)
// VERSION: 0.1.9
// PURPOSE: MS5611 Temperature & Atmospheric Pressure library for Arduino
//     URL:
//
// POST FORK HISTOTY
// 0.1.9  2019-01-17 modified for SPI
//                   replaced all floating point with integer arithmatic
//
// PRE-FORK HISTORY:
// 0.1.8  fix #109 incorrect constants (thanks to flauth)
// 0.1.7  revert double to float (issue 33)
// 0.1.6  2015-07-12 refactor
// 0.1.05 moved 6 float multiplies to init  [adds ~70 bytes !!!]
//        moved the MS5611_LIB_VERSION to PROGMEM
// 0.1.04 changed float to double (for platforms which support it)
//        changed divisions in multiplications
//        fixed uint32_t readADC()
//        reduced size of C array by 1 float
//        added second order temperature compensation
// 0.1.03 changed math to float [test version]
// 0.1.02 fixed bug return value read()
//        fixed bug #bits D2
//        added MS5611_READ_OK
//        added inline getters for temp & pres & lastresult.
//        adjusted delay's based on datasheet
//        merged convert functions
//        fixed offset in readProm()
// 0.1.01 small refactoring
// 0.1.00 added temperature and Pressure code
// 0.0.00 initial version by Rob Tillaart (15-okt-2014)
//
// Released to the public domain
//

#include "MS5611.h"

#include <SPI.h>
SPISettings settingsA(
    1000000, MSBFIRST,
    SPI_MODE0);  // define SPI settings; limit SPI communication to 1MHz

/////////////////////////////////////////////////////
//
// PUBLIC
//
MS5611::MS5611(uint8_t CSn) {
    _cspin = CSn;
    pinMode(_cspin, OUTPUT);
    digitalWrite(_cspin, HIGH);
    _temperature = -999;
    _pressure = -999;
    // init();
}

void MS5611::init() {
    reset();

    // Default values (C1 t/m C6 taken from example column in datasheet to test
    // calculations):
    C[0] = 1;
    C[1] = 40127;
    C[2] = 36924;
    C[3] = 23317;
    C[4] = 23282;
    C[5] = 33464;
    C[6] = 28312;
    C[7] = 0xF0F0;

    // Read all factory parameters C0 to C7 from PROM
    SPI.beginTransaction(settingsA);
    for (uint8_t reg = 0; reg < 8; reg++) {
        C[reg] = readProm(reg);
    }
    SPI.endTransaction();
}

int MS5611::read(uint8_t bits) {
    // VARIABLES NAMES BASED ON DATASHEET  <- Nice!
    convert(0x40, bits);
    uint32_t D1 = readADC();

    convert(0x50, bits);
    uint32_t D2 = readADC();

    // TODO the multiplications of these constants can be done in init()
    // but first they need to be verified.

    // TEMP & PRESS MATH - PAGE 7/20 of the datasheet
    //  - avoiding float type to make running on Tiny's not-impossible
    //  - running into issues with uint64_t so using uint32_t with adjustments
    //      (at the cost of reduced resolution for temperature).
    uint32_t Tref, dT;
    uint32_t dTC6;
    int32_t TEMP;
    Tref = C[5] * 256UL;
    if (D2 <
        Tref) {  // (to avoid signed int so we can bit-shift for divisioning)
        dT = Tref - D2;
        dTC6 = ((uint64_t)dT * (uint64_t)C[6]) >> 23;
        TEMP = 2000 - dTC6;
    } else {
        dT = D2 - Tref;
        dTC6 = ((uint64_t)dT * (uint64_t)C[6]) >> 23;
        TEMP = 2000 + dTC6;
    }

    // OFF = offT1 + TCO * dT = C2 * 2^16 + (C4 * dT ) / 2^7
    uint64_t offT1 = (uint64_t)C[2] << 16;
    uint64_t TCOdT = ((uint64_t)C[4] * (uint64_t)dT) >> 7;
    int64_t OFF;
    if (D2 < Tref) {
        OFF = offT1 - TCOdT;
    } else {
        OFF = offT1 + TCOdT;
    }

    // SENSE = sensT1 + TCS * dT = C1 * 2^15 + (C3 * dT ) / 2^8
    uint64_t sensT1 = (uint64_t)C[1] << 15;
    uint64_t TCSdT = ((uint64_t)C[3] * (uint64_t)dT) >> 8;
    int64_t SENS;
    if (D2 < Tref) {
        SENS = sensT1 - TCSdT;
    } else {
        SENS = sensT1 + TCSdT;
    }

    // SECOND ORDER COMPENSATION - PAGE 8/20 of the datasheet
    // COMMENT OUT < 2000 CORRECTION IF NOT NEEDED
    // NOTE TEMPERATURE IS IN 0.01 C
    uint32_t T2 = 0;
    uint32_t OFF2 = 0;
    uint32_t SENS2 = 0;
    if (TEMP < 2000) {
        uint64_t tSQ;
        T2 = ((uint64_t)dT * (uint64_t)dT) >> 31;
        tSQ = (int32_t)TEMP - 2000L;
        tSQ *= tSQ;
        OFF2 = (5ULL * (uint64_t)tSQ) >> 1;
        SENS2 = (5ULL * (uint64_t)tSQ) >> 2;
        // COMMENT OUT < -1500 CORRECTION IF NOT NEEDED
        if (TEMP < -1500) {
            tSQ = (int32_t)TEMP - 2000L;
            tSQ *= tSQ;
            OFF2 += 7ULL * (uint64_t)tSQ;
            SENS2 += (11ULL * (uint64_t)tSQ) >> 1;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    //
    // END SECOND ORDER COMPENSATION
    //

    int64_t P = (int64_t)D1 * SENS;
    P /= 2097152LL;
    P -= OFF;
    P /= 32768LL;

    _temperature = TEMP;
    _pressure = (uint32_t)P;

    return 0;
}

/////////////////////////////////////////////////////
//
// PRIVATE
//
void MS5611::reset() {
    SPI.begin();                      // Begin SPI
    SPI.beginTransaction(settingsA);  // start SPI transaction
    digitalWrite(_cspin, LOW);        // pull CS line low
    SPI.transfer(0x1E);               // send reset command
    delay(4);
    digitalWrite(_cspin, HIGH);  // pull CS line high
    SPI.endTransaction();        // end SPI transaction
}

void MS5611::convert(const uint8_t addr, uint8_t bits) {
    uint8_t del[5] = {1, 2, 3, 5,
                      10};  // array of MS5611 conversion time (in ms)
    bits = constrain(bits, 8, 12);
    uint8_t offset = (bits - 8) * 2;
    SPI.beginTransaction(settingsA);  // start SPI transaction
    digitalWrite(_cspin, LOW);        // pull CS line low
    SPI.transfer(addr + offset);      // send command
    delay(del[offset /
              2]);  // MS5611 needs some time for conversion; wait for this...
    digitalWrite(_cspin, HIGH);  // pull CS line high
    SPI.endTransaction();        // end SPI transaction
}

uint16_t MS5611::readProm(uint8_t reg) {
    // read two bytes from SPI and return accumulated value
    reg = min(reg, 7);
    uint8_t offset = reg * 2;
    uint16_t val = 0;
    digitalWrite(_cspin, LOW);       // pull CS line low
    SPI.transfer(0xA0 + offset);     // send command
    val = SPI.transfer(0x00) * 256;  // read 8 bits of data (MSB)
    val += SPI.transfer(0x00);       // read 8 bits of data (LSB)
    digitalWrite(_cspin, HIGH);      // pull CS line high
    return val;
}

uint32_t MS5611::readADC() {
    // read three bytes from SPI and return accumulated value
    uint32_t val = 0UL;
    SPI.beginTransaction(settingsA);     // start SPI transaction
    digitalWrite(_cspin, LOW);           // pull CS line low
    SPI.transfer(0x00);                  // send command
    val = SPI.transfer(0x00) * 65536UL;  // read 8 bits of data (MSB)
    val += SPI.transfer(0x00) * 256UL;   // read 8 bits of data
    val += SPI.transfer(0x00);           // read 8 bits of data (LSB)
    digitalWrite(_cspin, HIGH);          // pull CS line high
    SPI.endTransaction();                // end SPI transaction
    return val;
}

// END OF FILE
