#pragma once

#include "common/FifoBuffer.h"
#include "common/packet.h"

class RocketFSMBase {
   public:
    virtual void tickFSM() = 0;

    FSM_State getFSMState() const { return rocket_state_; }

   protected:
    FSM_State rocket_state_ = FSM_State::STATE_INIT;

   public:
    template <typename T, size_t count>
    static double getAverage(FifoBuffer<T, count>& buffer, double (*access_value)(T&), size_t start, size_t len) {
        // chMtxLock(&buffer.lock);
        T items[len];
        buffer.readSlice(items, start, len);
        double sum = 0.0;
        for (size_t i = 0; i < len; i++) {
            sum += access_value(items[i]);
        }
        // chMtxUnlock(&buffer.lock);
        return sum / (double)len;
    }

    template <typename T, size_t count>
    static double getSecondDerivativeAverage(FifoBuffer<T, count>& buffer, double (*access_value)(T&),
                                             systime_t (*access_time)(T&), size_t start, size_t len) {
        // chMtxLock(&buffer.lock);
        T items[len];
        buffer.readSlice(items, start, len);

        double derivatives[len - 1];
        for (size_t i = start; i < start + len - 1; i++) {
            double first = access_value(items[i]);
            double second = access_value(items[i + 1]);
            systime_t delta_t = access_time(items[i + 1]) - access_time(items[i]);
            derivatives[i - start] = (second - first) / (delta_t == 0 ? 0.02 : delta_t);
        }

        double second_derivatives[len - 2];
        for (size_t i = start; i < start + len - 2; i++) {
            double first = derivatives[i - start];
            double second = derivatives[i + 1 - start];
            systime_t delta_t = access_time(items[i + 1]) - access_time(items[i]);
            derivatives[i - start] = (second - first) / (delta_t == 0 ? 0.02 : delta_t);
        }
        // chMtxUnlock(&buffer.lock);

        double sum = 0.0;
        for (size_t i = 0; i < len - 2; i++) {
            sum += second_derivatives[i];
        }
        return sum / (double)(len - 2);
    }
};
