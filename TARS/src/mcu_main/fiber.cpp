#include "fiber.h"
constexpr size_t REAL_STACK_SIZE = 8372224;
#ifdef _WIN32
#include<windows.h>
void EmuSwitchToFiber(FiberHandle handle){
    SwitchToFiber(handle.handle);
}
FiberHandle EmuCreateFiber(size_t stack_size, ThreadFunc func, void* arg){
    void* handle = CreateFiber(REAL_STACK_SIZE, func, arg);
    return {.handle = handle, .real_stack_size = stack_size, .is_main = false};
}
FiberHandle EmuConvertThreadToFiber(){
    void* handle = ConvertThreadToFiber(nullptr);
    return {.handle = handle, .real_stack_size = 0, .is_main = true};
}
#else //_WIN32
#ifdef __APPLE__
#define _XOPEN_SOURCE
#endif
#include <ucontext.h>

#include <cassert>
#include <iostream>
#include <string.h>

constexpr const char* stack_end = "STACKEND";
static ucontext_t main_context;
static ucontext_t* current_context;
typedef void VoidFunc(void);
void EmuSwitchToFiber(FiberHandle handle) {
    ucontext_t* to_context = current_context;
    current_context = handle.handle;
    swapcontext(to_context, handle.handle);
}

FiberHandle EmuCreateFiber(size_t stack_size, ThreadFunc func, void* arg) {
    ucontext_t* context = new ucontext_t{};
    if (getcontext(context) == -1) {
        assert(!"Get context fail");
    }

    char* stack = new char[REAL_STACK_SIZE];
    context->uc_stack.ss_sp = stack;
    context->uc_stack.ss_size = REAL_STACK_SIZE;
    context->uc_link = &main_context;
    makecontext(context, (VoidFunc*)func, 1, arg);
    return {.handle = context, .real_stack_size = stack_size, .is_main=false};
}

// always main thread
FiberHandle EmuConvertThreadToFiber(void*) {
    current_context = &main_context;
    return {.handle = &main_context, .real_stack_size = 0, .is_main=true};
}
#endif