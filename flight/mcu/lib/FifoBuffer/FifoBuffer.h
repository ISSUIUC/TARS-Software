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
        size_t head_idx = tail_idx == 0 ? max_size - 1 : tail_idx - 1;
        read_to = arr[head_idx];
        chMtxUnlock(&lock);
        return true;
    }

    /**
     * @brief Reads a range of items into a passed array, which can be larger than the actual count of items.
     *
     * @param write_to An array of at least n items to write to
     * @param start The index to start reading from (inclusive)
     * @param end The index to stop reading at (exclusive)
     * @return How many items were actually read
     */
    size_t readSlice(T* write_to[], size_t start, size_t end) {
        chMtxLock(&lock);
        size_t i = 0;
        if (count < max_size) {
            size_t idx = start;
            while (idx != tail_idx && idx < end) {
                write_to[i++] = &arr[idx++];
            }
        } else {
            // if we are already wrapping, head == tail
            size_t idx = tail_idx + start;
            if (idx >= max_size) {
                idx -= max_size;
            }
            size_t end_idx = tail_idx + end;
            if (end_idx >= max_size) {
                end_idx -= max_size;
            }
            while (idx != tail_idx && idx < end_idx) {
                write_to[i++] = &arr[idx++];
            }
        }
        chMtxUnlock(&lock);
        return i;
    }

    // i'd've made the function pointers template parameters so we could guarantee inlining, but using lambdas as template
    //  parameters is a C++17 feature and platformio really doesn't want to use C++17
    double getAverage(double (*access_value)(T&), size_t start, size_t end) {
        T* items[end - start];
        readSlice(items, start, end);
        double sum = 0.0;
        for (size_t i = start; i < end; i++) {
            sum += access_value(*items[i]);
        }
        return sum / (double) (end - start);
    }

    double getSecondDerivativeAverage(double (*access_value)(T&), systime_t (*access_time)(T&), size_t start, size_t end) {
        T* items[end - start];
        readSlice(items, start, end);

        double derivatives[end - start - 1];
        for (size_t i = start; i < end - 1; i++) {
            double first = access_value(*items[i]);
            double second = access_value(*items[i + 1]);
            systime_t delta_t = access_time(*items[i + 1]) - access_time(*items[i]);
            derivatives[i] = (second - first) / (delta_t == 0 ? 0.02 : delta_t);
        }

        double second_derivatives[end - start - 2];
        for (size_t i = start; i < end - 2; i++) {
            double first = derivatives[i];
            double second = derivatives[i + 1];
            systime_t delta_t = access_time(*items[i + 1]) - access_time(*items[i]);
            derivatives[i] = (second - first) / (delta_t == 0 ? 0.02 : delta_t);
        }

        double sum = 0.0;
        for (size_t i = start; i < end - 2; i++) {
            sum += second_derivatives[i];
        }
        return sum / (double) (end - start - 2);
    }

    size_t count = 0;

private:
    /**
     * @brief index of the next slot to write to
     */
    size_t tail_idx = 0;
    /**
     * @brief total number of items that have been pushed
     */
    
    T arr[max_size]{};
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
