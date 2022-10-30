#ifndef TARS_SOFTWARE_FIFO_H
#define TARS_SOFTWARE_FIFO_H

#include <ChRt.h>

template <typename T, size_t max_size>
class FifoBuffer {
public:
    FifoBuffer() : length(0), head_idx(0), tail_idx(0), to_process_idx(0) {}

    bool push(T const& element) {
        lock();
        if (length == max_size) {
            // if length == max_size, then head_idx == tail_idx
            arr[tail_idx] = element;
            inc_idx(tail_idx);
            if (to_process_idx == head_idx) {
                to_process_idx = head_idx = tail_idx;
            } else {
                head_idx = tail_idx;
            }
        } else {
            arr[tail_idx] = element;
            inc_idx(tail_idx);
            length++;
        }
        unlock();
        return true;
    }

    bool pop(T* out) {
        // TODO remove this method entirely
        //   convert all usages to `next`
        lock();
        if (length == 0) {
            unlock();
            return false;
        } else {
            *out = arr[head_idx];
            if (to_process_idx == head_idx) {
                inc_idx(head_idx);
                to_process_idx = head_idx;
            }
            length--;
            unlock();
            return true;
        }
    }

    bool next(T& out) {
        lock();
        if (length == 0 || to_process_idx == tail_idx) {
            unlock();
            return false;
        } else {
            out = arr[to_process_idx];
            inc_idx(to_process_idx);
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

    /**
     * @brief Reads the most recent N items into a passed array (can be larger than the actual count of items).
     *
     * @param write_to An array of at least n items to write to
     * @param n How many items to read from the FIFO buffer
     * @return How many items were actually read
     */
    size_t recentN(T write_to[], size_t n) {
        // TODO change `n` to a template parameter on recent?
        if (n != 0) {
            size_t i = 0;
            size_t idx = head_idx;
            lock();
            while (idx != tail_idx && i < n) {
                write_to[i++] = arr[idx];
                inc_idx(idx);
            }
            unlock();
            return i;
        } else {
            return 0;
        }
    }

private:
    void inline inc_idx(size_t& idx) {
        idx++;
        if (idx == max_size) {
            idx = 0;
        }
    }

    size_t length;
    size_t head_idx;
    size_t tail_idx;
    size_t to_process_idx;
    T arr[max_size];

    MUTEX_DECL(lock_);
};


#endif  // TARS_SOFTWARE_FIFO_H
