#pragma once

#include <ChRt.h>
#include <SD.h>

#include "mcu_main/dataLog.h"
#include "mcu_main/error.h"

class SDLogger;
extern SDLogger sd_logger;

class SDLogger {
public:
    ErrorCode __attribute__((warn_unused_result)) init();

    void update();

private:
    template <typename T>
    void logData(T* data);

    DataLogQueue queue;
    File sd_file;
    size_t writes_since_flush = 0;
};
