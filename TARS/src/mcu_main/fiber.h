#pragma once

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include<cstddef>
//SwitchToFiber -> swapcontext
//CreateFiber -> make context
//ConvertThreadToFiber


typedef void* FiberHandle;
typedef void ThreadFunc(void*);

void SwitchToFiber(FiberHandle hanlde);

FiberHandle CreateFiber(size_t stack_size, ThreadFunc func, void* arg);

FiberHandle ConvertThreadToFiber(void*);
#endif