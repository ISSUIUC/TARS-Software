#pragma once

#include "mcu_main/debug.h"

#ifndef ENABLE_SILSIM_MODE
#include "ChRt.h"
#else
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iostream>

#define digitalWrite(pin, amount) (void)(0)
#define delay(time) (void)(0)

void createThread(const char* name, void fn(void*), size_t stack, void* arg);
void threadSleep(int32_t time_ms);
void threadYield();
uint32_t getTime();

void tone(uint8_t pin, uint16_t frequency);
void tone(uint8_t pin, uint16_t frequency, uint32_t duration);
void noTone(uint8_t pin);

struct Mutex {
   public:
    Mutex() : locked(false) {}

    void lock() {
        while (locked) {
            threadYield();
        }
    }

    void unlock() { locked = false; }

   private:
    volatile bool locked;
};

struct SerialPatch {
    void println(const char* s);

    template <typename T>
    void print(T t) {
        std::cout << t;
    }

    template <typename T, typename... Args>
    void print(T t, Args... args) {
        std::cout << t;

        print(args...);
    }

    void begin(int baudrate);
};

extern SerialPatch Serial;

typedef uint32_t systime_t;
typedef uint32_t sysinterval_t;

#define THD_FUNCTION(name, arg)      \
    const char* name##_name = #name; \
    static void name(void* arg)
#define THD_WORKING_AREA(name, size) uint8_t name[size];
#define NORMALPRIO 0
#define CH_CFG_ST_FREQUENCY 10000

#define chThdCreateStatic(wa_ptr, wa_size, prio, fn, arg) createThread(fn##_name, fn, wa_size, arg)
#define chThdSleepMilliseconds(time) threadSleep(time)
#define chThdYield() threadYield()
#define chVTGetSystemTime() (getTime() * 1000)
#define chBegin(f) f()

typedef uint64_t time_conv_t;
typedef uint32_t time_msecs_t;

#define MUTEX_DECL(name) Mutex name
#define chMtxLock(mtx) (mtx)->lock()
#define chMtxUnlock(mtx) (mtx)->unlock()
#define TIME_I2MS(interval)                                                                                   \
    (time_msecs_t)(                                                                                           \
        (((time_conv_t)(interval) * (time_conv_t)1000) + (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) / \
        (time_conv_t)CH_CFG_ST_FREQUENCY)

#define TIME_MS2I(msecs)                                                                              \
    ((sysinterval_t)((((time_conv_t)(msecs) * (time_conv_t)CH_CFG_ST_FREQUENCY) + (time_conv_t)999) / \
                     (time_conv_t)1000))

#define chSysLock() (void)(0)
#define chSysUnlock() (void)(0)

#endif