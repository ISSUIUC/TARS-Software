#pragma once

#include "mcu_main/Rt.h"

#ifdef ENABLE_SILSIM_MODE
#include <fstream>
#else
#include <SD.h>
#endif

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
#ifdef ENABLE_SILSIM_MODE
    std::ofstream file;
#else
    File file;
#endif
    size_t writes_since_flush = 0;
};
