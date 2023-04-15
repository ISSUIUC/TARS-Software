#pragma once

#include "mcu_main/debug.h"

#ifndef ENABLE_SILSIM_MODE
#include "ChRt.h"
#else
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iostream>


#define digitalWrite(pin, amount) (void) (0)
#define delay(time) (void) (0)

void createThread(const char* name, void fn(void*), size_t stack, void* arg);
void threadSleep(int32_t time_ms);
void threadYield();
uint32_t getTime();

struct Mutex {
public:
    Mutex() : locked(false) { }

    void lock() {
        while (locked) {
            threadYield();
        }
    }

    void unlock() {
        locked = false;
    }

private:
    volatile bool locked;
};

struct SerialPatch {
    void println(const char* s);

    template<typename T>
    void print(T t) {
        std::cout << t;
    }

    template<typename T, typename... Args>
    void print(T t, Args... args) {
        std::cout << t;

        print(args...);
    }

    void begin(int baudrate);
};

extern SerialPatch Serial;

typedef uint32_t systime_t;
typedef uint32_t sysinterval_t;

#define THD_FUNCTION(name, arg) const char* name##_name = #name; void name(void* arg)
#define THD_WORKING_AREA(name, size) uint8_t name[size];
#define NORMALPRIO 0

#define chThdCreateStatic(wa_ptr, wa_size, prio, fn, arg) createThread(fn##_name, fn, wa_size, arg)
#define chThdSleepMilliseconds(time) threadSleep(time)
#define chVTGetSystemTime() getTime()
#define chBegin(f) f()

#define MUTEX_DECL(name) Mutex name
#define chMtxLock(mtx) (mtx)->lock()
#define chMtxUnlock(mtx) (mtx)->unlock()
#define TIME_I2MS(t) ((t) / CLOCKS_PER_SEC * 1000)

#define chSysLock() (void) (0)
#define chSysUnlock() (void) (0)

#endif