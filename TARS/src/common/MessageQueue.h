#pragma once

#include <cstddef>

template <typename T, size_t max_count>
class MessageQueue {
   public:
    MessageQueue() = default;

    void push(T item) {
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
    }

    bool pop(T& item) {
        if (count == 0) {
            return false;
        }
        item = buffer[head_idx++];
        if (head_idx == max_count) {
            head_idx = 0;
        }
        count--;
        return true;
    }

   private:
    size_t count = 0;     // The number of items currently in the Queue.
    size_t head_idx = 0;  // The index to for the next pop to read from.
    size_t tail_idx = 0;  // The index to for the next push to write to.
    T buffer[max_count];
};