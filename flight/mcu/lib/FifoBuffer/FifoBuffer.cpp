//
// Created by 16182 on 9/30/2021.
//

//
// Created by 16182 on 9/28/2021.
//

#include "FifoBuffer.h"

#include <cstdint>
#include <cstring>
#include <mutex>

// only works with types that can be copied with memcpy
// switch to whatever mutex works on the platform
bool GenericFifoBuffer::push(void* element) {
    std::lock_guard<std::mutex> l(lock);
    if (capacity_ == cur_length_) {
        return false;
    }

    size_t byte_offset = data_size_ * tail_idx_;
    void* tail_ptr = (void*)((char*)arr_ + byte_offset);

    std::memcpy(tail_ptr, element, data_size_);

    tail_idx_ += 1;
    tail_idx_ %= capacity_;
    cur_length_ += 1;

    return true;
}

// returns false on failure
bool GenericFifoBuffer::pop(void* out) {
    std::lock_guard<std::mutex> l(lock);
    if (cur_length_ == 0) {
        return false;
    }

    size_t byte_offset = data_size_ * head_idx_;
    void* head_ptr = (void*)((char*)arr_ + byte_offset);
    std::memcpy(out, head_ptr, data_size_);

    head_idx_ += 1;
    head_idx_ %= capacity_;
    cur_length_ -= 1;

    return true;
}