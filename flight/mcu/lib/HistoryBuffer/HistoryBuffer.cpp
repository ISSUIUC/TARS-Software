#include "HistoryBuffer.h"

/*
History Fifo Buffer
*/

    HistoryBuffer::HistoryBuffer(){
        frontIndex = 0;
        endIndex = arrLength - 1;
    }

    // //debugging method, delete after
    // void printArray(){
    //     std::cout << "[ " << arr[0];
    //     for(int i = 1; i < sizeof(arr)/sizeof(arr[0]); i++){
    //         std::cout << " ," << arr[i];
    //     }
    //     std::cout << "]\n";
    // }

    // //debugging method, delete after
    // void printIndexes(){
    //     std::cout << "Start: " << frontIndex << "\n";
    //     std::cout << "End: " << endIndex << "\n"; 
    // }

    void HistoryBuffer::push(float val, float timestamp){
        //sets the value at the end of the queue to be equal to the desired push value
        arr[endIndex] = val;
        timestampArr[endIndex] = timestamp;

        //since our newest value is at the end index, it becomes our start index
        frontIndex = endIndex;

        //sets the new end index to the next oldest value which will be one less than the previous end index
        //if its currently 0, it needs to wrap around to the back
        if(endIndex == 0){
            endIndex = arrLength-1;
        }
        else{
            endIndex--;
        }
    }

    float HistoryBuffer::getCurrentAverage(){
        float count = 0.0;
        //want to average the first 50 values (start at 0, end at 49)
        for(int i = 0; i < arrLength/2; i++){
            //add the value of the queue at "i"th index from the front index (frontIndex + i)
            //if our index goes over, we mod it with the length to get it to wrap back around
            count += arr[(frontIndex + i)%arrLength];
        }
        return count/(arrLength/2);
    }

    float HistoryBuffer::getPastAverage(){
        float count = 0.0;
        //want to average the last 50 values (start at 50, end at 99)
        for(int i = arrLength/2; i < arrLength; i++){
            //add the value of the queue at "i"th index from the front index (frontIndex + i)
            //if our index goes over, we mod it with the length to get it to wrap back around
            count += arr[(frontIndex + i)%arrLength];
        }
        return count/(arrLength/2);
    }

    float HistoryBuffer::getCurrentSecondDerivativeAverage(){
        //we want to get velocity values from the altitude, which means that we need the change in altitidue over the change in time
        //we can calculate the change in velocity between the current value and the next value and divide it by the time 
        //the array will be 50-1 in length
        float firstDiff[(arrLength/2)-1];
        for(int i = 0; i < (arrLength/2)-1; i++){
            //we do arr[index] - arr[index+1] because the lower index is going to be the most recent value, which is why it goes first
            int currentIndex = (frontIndex + i)%arrLength;
            int pastIndex = (frontIndex + i+1)%arrLength;
            firstDiff[i] = (arr[currentIndex] - arr[pastIndex])/((timestampArr[currentIndex]-timestampArr[pastIndex])*0.001);
        }

        //we want to get acceleration from velocity, so we do the same thing but this time with the velocity array
        //the new array with be length 49-1
        float secondDiff[(arrLength/2)-2];
        for(int i = 0; i < (arrLength/2)-2; i++){
            secondDiff[i] = (firstDiff[i] - firstDiff[i+1])/((timestampArr[(frontIndex + i)%arrLength]-timestampArr[(frontIndex + i+1)%arrLength])*0.001);
        }

        //we then calculate the average of the acceleration array
        float avg = 0.0;
        for(int i = 0; i < (arrLength/2)-2; i++){
            avg += secondDiff[i];
        }
        return(avg/float(((arrLength/2)-2))); 
    }

    float HistoryBuffer::getPastSecondDerivativeAverage(){
        //same as current second derivative average, except we use indexes 50-99
        float firstDiff[(arrLength/2)-1];
        for(int i = arrLength/2; i < arrLength-1; i++){
            int currentIndex = (frontIndex + i)%arrLength;
            int pastIndex = (frontIndex + i+1)%arrLength;
            firstDiff[i-(arrLength/2)] = (arr[currentIndex] - arr[pastIndex])/((timestampArr[currentIndex]-timestampArr[pastIndex])*0.001);
        }

        float secondDiff[(arrLength/2)-2];
        for(int i = 0; i < (arrLength/2)-2; i++){
            secondDiff[i] = (firstDiff[i] - firstDiff[i+1])/((timestampArr[(frontIndex + i)%arrLength]-timestampArr[(frontIndex + i+1)%arrLength])*0.001);
        }

        float avg = 0.0;
        for(int i = 0; i < (arrLength/2)-2; i++){
            avg += secondDiff[i];
        }
        return(avg/float(((arrLength/2)-2))); 
    }