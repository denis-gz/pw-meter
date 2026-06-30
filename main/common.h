#pragma once

#include "sdkconfig.h"
#include <array>
#include <cstddef>

#define nameof(x) #x

#define countof(ARRAY) (sizeof(ARRAY) / sizeof(ARRAY[0]))

extern const char* const TAG;

template <typename F, class T, typename R, typename... Args>
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

template<size_t size>
using char_array_t = std::array<char, size>;

typedef char_array_t<17>  display_line_t;
typedef char_array_t<256> string_t;
