#ifndef _WIN32
#include "fiber.h"

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
void SwitchToFiber(FiberHandle handle) {
    ucontext_t* to_context = current_context;
    current_context = (ucontext_t*)handle;
    if (current_context != &main_context &&
        memcmp((char*)current_context->uc_stack.ss_sp + current_context->uc_stack.ss_size, stack_end, 8) != 0) {
        std::cerr << "stack corruption\n";
        exit(1);
    }
    swapcontext(to_context, (ucontext_t*)handle);
}

FiberHandle CreateFiber(size_t stack_size, ThreadFunc func, void* arg) {
    ucontext_t* context = new ucontext_t{};
    if (getcontext(context) == -1) {
        assert(!"Get context fail");
    }

    char* stack = new char[stack_size + 8];
    memcpy(stack + stack_size, stack_end, 8);
    context->uc_stack.ss_sp = stack;
    context->uc_stack.ss_size = stack_size;
    context->uc_link = &main_context;
    makecontext(context, (VoidFunc*)func, 1, arg);

    return context;
}

// always main thread
FiberHandle ConvertThreadToFiber(void*) {
    current_context = &main_context;
    return &main_context;
}
#endif