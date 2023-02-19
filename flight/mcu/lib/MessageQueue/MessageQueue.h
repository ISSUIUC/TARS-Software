#pragma once

#include <ChRt.h>

template<typename T, size_t max_count>
class MessageQueue {
public:
    MUTEX_DECL(lock);

    MessageQueue() = default;

    void push(T item) {
        chMtxLock(&lock);
        buffer[tail_idx++] = item;
        if (tail_idx == max_count) {
            tail_idx = 0;
        }
        if (count == max_count) {
            head_idx++;
            if (head_idx == max_count) {
                head_idx = 0;
            }
        } else {
            count++;
        }
        chMtxUnlock(&lock);
    }

    bool pop(T& item) {
        chMtxLock(&lock);
        if (count == 0) {
            chMtxUnlock(&lock);
            return false;
        }
        item = buffer[head_idx++];
        if (head_idx == max_count) {
            head_idx = 0;
        }
        count--;
        chMtxUnlock(&lock);
        return true;
    }

private:
    size_t count = 0;      // The number of items currently in the Queue.
    size_t head_idx = 0;   // The index to for the next pop to read from.
    size_t tail_idx = 0;   // The index to for the next push to write to.
    T buffer[max_count];
};