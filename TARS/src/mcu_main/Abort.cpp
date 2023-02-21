#include "mcu_main/Abort.h""

bool is_aborted = false;

void startAbort() { is_aborted = true; }

bool isAborted() { return is_aborted; }
