#pragma once

#include "dataLog.h"
#include <ChRt.h>
#include <SD.h>

class SDLogger;
extern SDLogger sd_logger;

class SDLogger {
public:
    explicit SDLogger(DataLogBuffer& buffer);

    void init();

    void update();

private:
    template<typename T>
    void logData(T* data);

    DataLogView view;
    File sd_file;
    size_t writes_since_flush = 0;
};
