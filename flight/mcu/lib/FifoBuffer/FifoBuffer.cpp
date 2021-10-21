//
// Created by 16182 on 9/30/2021.
//

//
// Created by 16182 on 9/28/2021.
//

#include "FifoBuffer.h"

#include <cstdint>
#include <cstring>

#ifdef COMPILE_LOCAL
#include <mutex>
#endif

#ifdef COMPILE_TARGET
#include <ChRt.h>
#endif

// only works with types that can be copied with memcpy
bool GenericFifoBuffer::push(void* element) {
#ifdef COMPILE_LOCAL
    // std::lock_guard locks the mutex when it is created and unlocks it is
    // destructed
    std::lock_guard<std::mutex> l(lock_);
#endif

#ifdef COMPILE_TARGET
    chMtxLock(lock_);
#endif

    if (capacity_ == cur_length_) {
#ifdef COMPILE_TARGET
        chMtxUnlock(lock_);
#endif
        return false;
    }

    size_t byte_offset = data_size_ * tail_idx_;
    void* tail_ptr = (void*)((char*)arr_ + byte_offset);

    std::memcpy(tail_ptr, element, data_size_);

    tail_idx_ += 1;
    tail_idx_ %= capacity_;
    cur_length_ += 1;

#ifdef COMPILE_TARGET
    chMtxUnlock(lock_);
#endif

    return true;
}

// returns false on failure
bool GenericFifoBuffer::pop(void* out) {
#ifdef COMPILE_LOCAL
    std::lock_guard<std::mutex> l(lock_);
#endif

#ifdef COMPILE_TARGET
    chMtxLock(lock_);
#endif

    if (cur_length_ == 0) {
#ifdef COMPILE_TARGET
        chMtxUnlock(lock_);
#endif
        return false;
    }

    size_t byte_offset = data_size_ * head_idx_;
    void* head_ptr = (void*)((char*)arr_ + byte_offset);
    std::memcpy(out, head_ptr, data_size_);

    head_idx_ += 1;
    head_idx_ %= capacity_;
    cur_length_ -= 1;

#ifdef COMPILE_TARGET
    chMtxUnlock(lock_);
#endif

    return true;
}
