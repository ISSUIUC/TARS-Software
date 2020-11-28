#ifndef TEENSYSHARED_H
#define TEENSYSHARED_H

#include <inttypes.h>

//data struct for FSM to send to ballValve_THD
struct ballValve_Message{
  bool isOpen;
  int timeStamp; //timeStamp is set when open or close command is sent.
  //more data can be added as needed
};

//data struct for hybridPT_THD to send pressure transducer data to FSM
struct pressureData{
  int PT1;
  int PT2;
  int PT3;
  int timeStamp; //timeStamp is set when pressure data is read.
  //more data can be added as needed
};

float ptConversion(uint16_t);

#endif