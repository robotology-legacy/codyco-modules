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

#include "LockGuard.h"
#include <yarp/os/Mutex.h>

namespace codyco {
    
    LockGuard::LockGuard(yarp::os::Mutex& mutex)
    : m_mutex(mutex)
    {
        m_mutex.lock();
    }
    
    LockGuard::~LockGuard()
    {
        m_mutex.unlock();
    }
    
    LockGuard::LockGuard(const LockGuard& lg)
    : m_mutex(lg.m_mutex) { }
    LockGuard& LockGuard::operator=(const LockGuard&) { return *this; }
}
