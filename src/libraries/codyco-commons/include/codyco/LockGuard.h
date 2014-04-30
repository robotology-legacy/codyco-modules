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

#ifndef CODYCOLIB_LOCKGUARD_H
#define CODYCOLIB_LOCKGUARD_H

namespace yarp {
    namespace os {
        class Mutex;
    }
}

namespace codyco {
    
    /** @class provides a lock guard around a Yarp Mutex.
     * This class implements the same behaviour of std::unique_lock in C++11
     */
    class LockGuard
    {
    public:
        /**
         * Obtain a lock on the passed mutex.
         * @param mutex mutex on which lock the current thread.
         */
        explicit LockGuard(yarp::os::Mutex& mutex);
        
        /**
         * Releases the lock on the mutex
         */
        ~LockGuard();
        
    private:
        //disable copy constructor and assignment operator
        LockGuard(const LockGuard&);
        LockGuard& operator=(const LockGuard&);
        
        yarp::os::Mutex& m_mutex;
    };
}
#endif
