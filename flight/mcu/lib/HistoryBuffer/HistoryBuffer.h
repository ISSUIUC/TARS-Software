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
template <size_t size>
class HistoryBuffer{
    private:
        float arr[size];
        systime_t timestampArr[size];

        int frontIndex;
        int endIndex;
    
    public:
        HistoryBuffer(){
            frontIndex = 0;
            endIndex = size - 1;
        }

        void push(float val, float timestamp){
            //sets the value at the end of the queue to be equal to the desired push value
            arr[endIndex] = val;
            timestampArr[endIndex] = timestamp;

            //since our newest value is at the end index, it becomes our start index
            frontIndex = endIndex;

            //sets the new end index to the next oldest value which will be one less than the previous end index
            //if its currently 0, it needs to wrap around to the back
            if(endIndex == 0){
                endIndex = size-1;
            }
            else{
                endIndex--;
            }
        }

        float getCurrentAverage(){
            float count = 0.0;
            //want to average the first 50 values (start at 0, end at 49)
            for(unsigned i = 0; i < size/2; i++){
                //add the value of the queue at "i"th index from the front index (frontIndex + i)
                //if our index goes over, we mod it with the length to get it to wrap back around
                count += arr[(frontIndex + i)%size];
            }
            return count/(size/2);
        }

        float getPastAverage(){
            float count = 0.0;
            //want to average the last 50 values (start at 50, end at 99)
            for(unsigned i = size/2; i < size; i++){
                //add the value of the queue at "i"th index from the front index (frontIndex + i)
                //if our index goes over, we mod it with the length to get it to wrap back around
                count += arr[(frontIndex + i)%size];
            }
            return count/(size/2);
        }

        float getCurrentSecondDerivativeAverage(){
            //we want to get velocity values from the altitude, which means that we need the change in altitidue over the change in time
            //we can calculate the change in velocity between the current value and the next value and divide it by the time 
            //the array will be 50-1 in length
            float firstDiff[(size/2)-1];
            for(unsigned i = 0; i < (size/2)-1; i++){
                //we do arr[index] - arr[index+1] because the lower index is going to be the most recent value, which is why it goes first
                int currentIndex = (frontIndex + i)%size;
                int pastIndex = (frontIndex + i+1)%size;

                if(timestampArr[currentIndex]-timestampArr[pastIndex] != 0){
                    firstDiff[i] = (arr[currentIndex] - arr[pastIndex])/(TIME_I2MS(timestampArr[currentIndex]-timestampArr[pastIndex])*0.001);
                }
                else{
                    firstDiff[i] = (arr[currentIndex] - arr[pastIndex])/(0.02);
                }
                
            }

            //we want to get acceleration from velocity, so we do the same thing but this time with the velocity array
            //the new array with be length 49-1
            float secondDiff[(size/2)-2];
            for(unsigned i = 0; i < (size/2)-2; i++){
                if(timestampArr[(frontIndex + i)%size]-timestampArr[(frontIndex + i+1)%size] != 0){
                    secondDiff[i] = (firstDiff[i] - firstDiff[i+1])/(TIME_I2MS(timestampArr[(frontIndex + i)%size]-timestampArr[(frontIndex + i+1)%size])*0.001);
                }
                else{
                    secondDiff[i] = (firstDiff[i] - firstDiff[i+1])/(0.02);
                }

            }

            //we then calculate the average of the acceleration array
            float avg = 0.0;
            for(unsigned i = 0; i < (size/2)-2; i++){
                
                avg += secondDiff[i];
            }
            return(avg/float(((size/2)-2))); 
        }

        float getPastSecondDerivativeAverage(){
            //same as current second derivative average, except we use indexes 50-99
            float firstDiff[(size/2)-1];
            for(unsigned i = size/2; i < size-1; i++){
                int currentIndex = (frontIndex + i)%size;
                int pastIndex = (frontIndex + i+1)%size;
                if(timestampArr[currentIndex]-timestampArr[pastIndex] != 0){
                    firstDiff[i-(size/2)] = (arr[currentIndex] - arr[pastIndex])/(TIME_I2MS(timestampArr[currentIndex]-timestampArr[pastIndex])*0.001);
                }
                else{
                    firstDiff[i-(size/2)] = (arr[currentIndex] - arr[pastIndex])/(0.02);
                }
            }

            float secondDiff[(size/2)-2];
            for(unsigned i = 0; i < (size/2)-2; i++){
                if(timestampArr[(frontIndex + i)%size]-timestampArr[(frontIndex + i+1)%size] != 0){
                    secondDiff[i] = (firstDiff[i] - firstDiff[i+1])/(TIME_I2MS(timestampArr[(frontIndex + i)%size]-timestampArr[(frontIndex + i+1)%size])*0.001);
                }
                else{
                    secondDiff[i] = (firstDiff[i] - firstDiff[i+1])/(0.02);
                }
            }
    
            float avg = 0.0;
            for(unsigned i = 0; i < (size/2)-2; i++){
                avg += secondDiff[i];
            }
            return(avg/float(((size/2)-2))); 
        }
};
#endif