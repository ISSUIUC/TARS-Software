#pragma once

#include <cstddef>

template <typename T, size_t max_size>
class FifoBuffer {
   public:
    FifoBuffer() = default;

    bool push(T const& element) {
        arr[tail_idx++] = element;
        if (tail_idx == max_size) {
            tail_idx = 0;
        }
        if (count < max_size) count++;
        return true;
    }

    bool read(T& item) {
        if (count == 0) {
            return false;
        }
        item = arr[newest()];
        return true;
    }

    /**
     * @brief Reads a range of items into a passed array, which can be larger than the actual count of items.
     *
     * @param write_to An array of at least n items to write to
     * @param start The index to start reading from (inclusive)
     * @param length the number of items to read
     * @return How many items were actually read
     */
    // TODO make this function return a std::array?
    size_t readSlice(T write_to[], size_t start, size_t len) {
        size_t i = 0;
        size_t idx = head() + start;
        while (i < len) {
            write_to[i++] = arr[idx++];
            if (idx == max_size) {
                idx = 0;
            }
        }
        return i;
    }

   private:
    /**
     * @brief Returns the head index. Do not use if count == 0, and always lock before using.
     *
     * @return the index of the oldest element you can read from
     */
    size_t head() {
        if (tail_idx < count) {
            return tail_idx + max_size - count;
        } else {
            return tail_idx - count;
        }
    }

    /**
     * @brief Returns the index of the newest element. Do not use if count == 0, and always lock before using.
     *
     * @return the index of the newest element you can read from
     */
    size_t newest() {
        if (tail_idx == 0) {
            return max_size - 1;
        } else {
            return tail_idx - 1;
        }
    }

    size_t tail_idx = 0;  // index of the next slot to write to
    size_t count = 0;     // number of items currently in the buffer

    T arr[max_size];
};
