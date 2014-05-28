/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Francesco Romano
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef CODYCOLIB_UTILS_H
#define CODYCOLIB_UTILS_H

#include <cstdio>

//this is standard in C99 and C++11, but not on previous version of C++. GCC, Clang and MSVS should support it anyway
#define DLOG(format, ...) \
    printf(("[%s:%d] " format), __FILE__, __LINE__, ##__VA_ARGS__)

#ifndef FUNCTION_NAME
#if defined(_MSC_VER)
#define FUNCTION_NAME __FUNCTION__
#elif defined(__GNUC__) || defined(__clang__)
#define FUNCTION_NAME __PRETTY_FUNCTION__
#else
#define FUNCTION_NAME ""
#endif
#endif

#ifndef CODYCO_DEPRECATED(message)
#if defined(__clang__)
#define CODYCO_DEPRECATED(message)  __attribute__((deprecated(message)))
#elif defined(__GNUC__)
#define CODYCO_DEPRECATED(message)  __attribute__ ((deprecated))
#elif defined (_MSC_VER)
#define CODYCO_DEPRECATED(message) __declspec(deprecated(message))
#else
#define CODYCO_DEPRECATED(message)
#endif
#endif

namespace codyco {}


#endif
