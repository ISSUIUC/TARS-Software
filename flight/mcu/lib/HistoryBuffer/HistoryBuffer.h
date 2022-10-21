#ifndef TARS_SOFTWARE_HISTORY_BUFFER
#define TARS_SOFTWARE_HISTORY_BUFFER

#include <cstdint>
#include <cstring>

#ifdef COMPILE_LOCAL
#include <mutex>
#endif

#include <ChRt.h>


/*
History Fifo Buffer
*/
class HistoryBuffer{
    private:
        //If not 100, make sure that it's always even
        const int arrLength = 100;

        float arr[100];
        systime_t timestampArr[100];

        int frontIndex;
        int endIndex;
    
    public:
        HistoryBuffer();
        void push(float val, float timestamp);
        float getCurrentAverage();
        float getPastAverage();
        float getCurrentSecondDerivativeAverage();
        float getPastSecondDerivativeAverage();
};
#endif