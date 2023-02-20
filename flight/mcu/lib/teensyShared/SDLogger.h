#pragma once

#include <ChRt.h>
#include <SD.h>

#include "dataLog.h"

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
