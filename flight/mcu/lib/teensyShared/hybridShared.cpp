#include "hybridShared.h"

/** ptConversion
 * DESCRIPTION: Converts raw sensor data from hybrid engine pressure transducers
 * into pressure in pascals. NOTE: Does not currently do anything. Needs
 * calibration data from pressure testing. INPUTS: rawData - 13 bit data
 * recieved from Teensy's ADC to convert into pressure. RETURNS: pPascals -
 * Floating point representation of pressure data in pascals.
 */
float ptConversion(uint16_t rawData) {
    // placeholder until we get calibration data to do conversion. TODO:
    // implement conversion
    float pPascals = rawData;
    return pPascals;
}