#include"unix_fiber.h"

#include <ucontext.h>
#include <cassert>
#include<iostream>

static ucontext_t main_context;
static ucontext_t *current_context;
typedef void VoidFunc(void);
void SwitchToFiber(FiberHandle handle){
    ucontext_t* to_context = current_context;
    current_context = (ucontext_t*)handle;
    std::cout << "FROM " << to_context << " TO " << (ucontext_t*)handle << '\n';
    swapcontext(to_context, (ucontext_t*)handle);
}

FiberHandle CreateFiber(size_t stack_size, ThreadFunc func, void* arg){
    ucontext_t* context = new ucontext_t{};
    if(getcontext(context) == -1){
        assert(!"Get context fail");
    }

    context->uc_stack.ss_sp = new char[stack_size];
    context->uc_stack.ss_size = stack_size;
    context->uc_link = &main_context;
    makecontext(context, (VoidFunc*)func, 1, arg);

    return context;
}

//always main thread
FiberHandle ConvertThreadToFiber(void*){
    current_context = &main_context;
    return &main_context;
}