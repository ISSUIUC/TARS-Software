//
// Created by 16182 on 9/27/2021.
//

#ifndef TARS_SOFTWARE_FIFO_H
#define TARS_SOFTWARE_FIFO_H

#include <cstdint>
#include <cstring>
#include <optional>
#include <mutex>

// only works with types that can be copied with memcpy
// switch to whatever mutex works on the platform
class GenericFifoBuffer {
   public:
    // buff_size = size of array in elements
    // data_size = size of array element in bytes
    GenericFifoBuffer(void* arr, uint16_t buff_size, uint16_t data_size) {
        capacity_ = buff_size;
        cur_length_ = 0;
        head_idx_ = 0;
        tail_idx_ = 0;
        data_size_ = data_size;
        arr_ = arr;
    }

    bool push(void* element);

    // returns false on failure
    bool pop(void* out);

   private:
    uint16_t capacity_, cur_length_, head_idx_, tail_idx_, data_size_;
    void* arr_;
    std::mutex lock;
};

template<typename T, size_t max_size>
class FifoBuffer{
    static_assert(std::is_trivially_copyable<T>::value, "Only trivially copyable types are allowed");
public:
    FifoBuffer(): buffer(arr, max_size, sizeof(T)){

    }

    bool push(T element){
        return buffer.push(&element);
    }

    std::optional<T> pop(){
        T ele;
        bool popped = buffer.pop(&ele);
        if(popped){
            return ele;
        } else {
            return std::nullopt;
        }
    }

private:
    GenericFifoBuffer buffer;
    T arr[max_size]{};
};

#endif  // TARS_SOFTWARE_FIFO_H
