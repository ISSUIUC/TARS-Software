#pragma once

#include <ChRt.h>
#include <SD.h>

#include "mcu_main/dataLog.h"

// Latest git hash of packet.h (see gen/git_hash.c)
extern char PACKET_H_GIT_HASH[41];

class SDLogger;
extern SDLogger sd_logger;

class SDLogger {
   public:
    void init();

    void update();

   private:
    template <typename T>
    void logData(T* data);

    DataLogQueue queue;
    File sd_file;
    size_t writes_since_flush = 0;
};
