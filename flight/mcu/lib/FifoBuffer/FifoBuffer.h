#ifndef TARS_SOFTWARE_FIFO_H
#define TARS_SOFTWARE_FIFO_H

#include <cstdint>
#include <cstring>
#include <ChRt.h>
#include <iterator>

template <typename T, size_t max_size>
class FifoBuffer {
public:
    struct FifoIterator {
    public:
        using iterator_category = std::input_iterator_tag;
        using difference_type = int64_t;
        using value_type = T;
        using pointer_type = T*;
        using reference_type = T&;

        FifoIterator(T* buf, size_t idx) : buf(buf), idx(idx) {}

        T& operator*() const {
            return buf[idx];
        }
        T* operator->() {
            return &buf[idx];
        }
        FifoIterator& operator++() {
            idx = (idx + 1) % max_size;
            return *this;
        }
        FifoIterator operator++(int) {
            FifoIterator tmp = *this;
            ++(*this);
            return tmp;
        }
        friend bool operator== (const FifoIterator& a, const FifoIterator& b) {
            return a.buf == b.buf && a.idx == b.idx;
        }
        friend bool operator!= (const FifoIterator& a, const FifoIterator& b) {
            return a.buf != b.buf || a.idx != b.idx;
        }

    private:
        T* buf;
        size_t idx;
    };


    FifoBuffer() : length(0), head_idx(0), tail_idx(0) {}

    bool push(T& element) {
        lock();
        if (length == max_size) {
            // if length == max_size, then head_idx == tail_idx
            arr[tail_idx] = element;
            head_idx = tail_idx = (head_idx + 1) % max_size;
        } else {
            arr[tail_idx] = element;
            tail_idx = (tail_idx + 1) % max_size;
            length++;
        }
        unlock();
        return true;
    }

    bool pop(T* out) {
        lock();
        if (length == 0) {
            unlock();
            return false;
        } else {
            *out = arr[head_idx];
            head_idx = (head_idx + 1) % max_size;
            length--;
            unlock();
            return true;
        }
    }

    void lock() {
        chMtxLock(&lock_);
    }

    void unlock() {
        chMtxUnlock(&lock_);
    }

    // before iterating, make sure to lock (and don't do pushes or pops while iterating)
    FifoIterator begin() {
        return FifoIterator { arr, head_idx };
    }

    FifoIterator end() {
        return FifoIterator { arr, tail_idx };
    }
    /**
     * @brief Reads the most recent N items into a passed array (can be larger than the actual count of items).
     *
     * @param write_to An array of at least n items to write to
     * @param n How many items to read from the FIFO buffer
     * @return How many items were actually read
     */
    size_t recentN(T write_to[], size_t n) {
        size_t i = 0;
        if (n != 0) {
            lock();
            for (T& item : this) {
                write_to[i] = item;
                if (i++ == n) break;
            }
            unlock();
        }
        return i;
    }

private:
    size_t length;
    size_t head_idx;
    size_t tail_idx;
    T arr[max_size];

    MUTEX_DECL(lock_);
};


#endif  // TARS_SOFTWARE_FIFO_H
