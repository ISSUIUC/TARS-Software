#pragma once
typedef void ThreadFunc(void*);
struct FiberHandle;
void EmuSwitchToFiber(FiberHandle handle);
FiberHandle EmuCreateFiber(size_t stack_size, ThreadFunc func, void* arg);
FiberHandle EmuConvertThreadToFiber();
#if defined(_WIN32) || defined(_WIN64)
struct FiberHandle {
    void* handle;
    size_t real_stack_size;
    bool is_main;
};
#else
#include <ucontext.h>
struct FiberHandle {
    ucontext_t * handle;
    size_t real_stack_size;
    bool is_main;
};
#endif
