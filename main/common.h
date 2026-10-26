#pragma once

#include "sdkconfig.h"
#include <esp_heap_caps.h>
#include <array>
#include <cstddef>
#include <new>
#include <utility>

#define nameof(x) #x

#define countof(ARRAY) (sizeof(ARRAY) / sizeof(ARRAY[0]))

extern const char* const TAG;

template<typename F, typename T, typename R, typename... Args>
F member_cast(R (T::*member_ptr)(Args...))
{
    union {
        R (T::*m_ptr)(Args...);
        F f_ptr;
    }
    cast {
        .m_ptr = member_ptr
    };

    return cast.f_ptr;
}

template<typename T, typename... Args>
T* psram_new(Args&&... args)
{
    void* p = heap_caps_malloc(sizeof(T), MALLOC_CAP_SPIRAM);
    return p ? new (p) T(std::forward<Args>(args)...) : nullptr;
}

template<typename T>
void psram_delete(T* p)
{
    if (p) {
        p->~T();
        heap_caps_free(p);
    }
}

template<size_t size>
using char_array_t = std::array<char, size>;

typedef char_array_t<17>  display_line_t;
typedef char_array_t<256> string_t;
