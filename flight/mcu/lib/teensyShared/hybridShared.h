#ifndef HYBRIDSHARED_H
#define HYBRIDSHARED_H

#include <inttypes.h>

// data struct for hybridPT_THD to send pressure transducer data to FSM
struct pressureData {
    float PT1;
    float PT2;
    float PT3;
    int timeStamp;  // timeStamp is set when pressure data is read.
    // more data can be added as needed
};

float ptConversion(uint16_t);

#endif