//
// Created by 16182 on 9/27/2021.
//

#ifndef TARS_SOFTWARE_FIFO_H
#define TARS_SOFTWARE_FIFO_H

#include<cstdint>
#include<cstring>
#include<mutex>

// only works with types that can be copied with memcpy
// switch to whatever mutex works on the platform
class FifoBuffer{
   public:
    // buff_size = size of array in elements
    // data_size = size of array element in bytes
    FifoBuffer(void * arr, uint16_t buff_size, uint16_t data_size){
        capacity_ = buff_size;
        cur_length_ = 0;
        head_idx_ = 0;
        tail_idx_ = 0;
        data_size_ = data_size;
        arr_ = arr;
    }

    bool push(void * element){
        std::lock_guard<std::mutex> l(lock);
        if(capacity_ == cur_length_){
            return false;
        }

        size_t byte_offset = data_size_ * tail_idx_;
        void * tail_ptr = (void*)((char*)arr_ + byte_offset);

        std::memcpy(tail_ptr, element, data_size_);

        tail_idx_ += 1;
        tail_idx_ %= capacity_;
        cur_length_ += 1;

        return true;
    }

    //returns false on failure
    bool pop(void * out){
        std::lock_guard<std::mutex> l(lock);
        if(cur_length_ == 0){
            return false;
        }

        size_t byte_offset = data_size_ * head_idx_;
        void * head_ptr = (void*)((char*)arr_ + byte_offset);
        std::memcpy(out, head_ptr, data_size_);

        head_idx_ += 1;
        head_idx_ %= capacity_;
        cur_length_ -= 1;

        return true;
    }

   private:
    uint16_t capacity_, cur_length_, head_idx_, tail_idx_, data_size_;
    void * arr_;
    std::mutex lock;
};

#endif  // TARS_SOFTWARE_FIFO_H
