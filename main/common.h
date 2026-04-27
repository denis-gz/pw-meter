#pragma once

#include "sdkconfig.h"

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
