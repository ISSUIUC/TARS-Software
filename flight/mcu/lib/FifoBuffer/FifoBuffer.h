#ifndef TARS_SOFTWARE_FIFO_H
#define TARS_SOFTWARE_FIFO_H

#include <ChRt.h>

template<typename T, size_t max_size>
class FifoView;

template<typename T, size_t max_size>
class FifoBuffer {
public:
    friend class FifoView<T, max_size>;

    MUTEX_DECL(lock);

    FifoBuffer() = default;

    bool push(T const& element) {
        chMtxLock(&lock);
        arr[tail_idx++] = element;
        if (tail_idx == max_size) {
            tail_idx = 0;
        }
        count++;
        chMtxUnlock(&lock);
        return true;
    }

    bool read(T& read_to) {
        chMtxLock(&lock);
        if (count < 1) {
            chMtxUnlock(&lock);
            return false;
        }
        read_to = arr[head()];
        chMtxUnlock(&lock);
        return true;
    }

    size_t head() {
        if (count <= max_size) {
            return 0;
        } else {
            return tail_idx;
        }
    }

    /**
     * @brief Reads a range of items into a passed array, which can be larger than the actual count of items.
     *
     * Note that this function returns pointers to elements of the buffer, so this isn't thread safe. Lock and unlock
     * the mutex yourself.
     *
     * @param write_to An array of at least n items to write to
     * @param start The index to start reading from (inclusive)
     * @param length the number of items to read
     * @return How many items were actually read
     */
    // TODO make this function return a std::array?
    size_t readSlice(T* write_to[], size_t start, size_t len) {
//        chMtxLock(&lock);
        size_t i = 0;
        size_t idx = head() + start;
        while (i < len) {
            write_to[i++] = &arr[(idx++) % max_size];  // TODO optimize this later
        }
//        chMtxUnlock(&lock);
        return i;
    }

private:
    size_t count = 0;

    /**
     * @brief index of the next slot to write to
     */
    size_t tail_idx = 0;
    /**
     * @brief total number of items that have been pushed
     */
    
    T arr[max_size];
};

template<typename T, size_t max_size>
class FifoView {
public:
    friend class FifoBuffer<T, max_size>;

    explicit FifoView(FifoBuffer<T, max_size>& observed_) : observed(observed_) {}

    bool next(T& out) {
        chMtxLock(&observed.lock);
        if (observed.count == read_count) {
            chMtxUnlock(&observed.lock);
            return false;
        }

        while (read_count + max_size < observed.count) {
            read_count++;
            idx++;
            if (idx == max_size) {
                idx = 0;
            }
        }

        out = observed.arr[idx++];
        if (idx == max_size) {
            idx = 0;
        }
        read_count++;

        chMtxUnlock(&observed.lock);
        return true;
    }

private:
    FifoBuffer<T, max_size>& observed;
    size_t read_count = 0;
    size_t idx = 0;
};


#endif  // TARS_SOFTWARE_FIFO_H
