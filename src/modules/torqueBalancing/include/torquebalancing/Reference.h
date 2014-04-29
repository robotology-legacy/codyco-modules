/**
 * Copyright (C) 2014 CoDyCo
 * @author: Francesco Romano
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

#ifndef REFERENCE_H
#define REFERENCE_H

#include <yarp/os/Mutex.h>
#include "Eigen/Core"


namespace codyco {
    namespace torquebalancing {
        
        template <class T>
        class Reference
        {
        public:
            Reference();
            ~Reference();
            
            T& value();
            void setValue(T& _value);
            
        private:
            yarp::os::Mutex m_lock;
            T m_value;
        };
        
        struct ControllerReferences
        {
        public:
            
            typedef Eigen::Matrix<double, 3, 1> COMAccelerationType;
            typedef Eigen::Matrix<double, 14, 1> HandsPositionType;
            typedef Eigen::Matrix<double, 12, 1> HandsForceType;
            
            Reference<Eigen::Matrix<double, 3, 1> > desiredCOMAcceleration;
            Reference<Eigen::Matrix<double, 14, 1> > desiredHandsPosition;
            Reference<Eigen::Matrix<double, 12, 1> > desiredHandsForce;
        };
        
        
        
//#include "Reference.hpp"
#include "config.h"
        
        template<class T>
        Reference<T>::Reference() {}
        
        template<class T>
        Reference<T>::~Reference() {}
        
        template<class T>
        T& Reference<T>::value()
        {
            codyco::torquebalancing::LockGuard guard(m_lock);
            return m_value;
        }
        
        template<class T>
        void Reference<T>::setValue(T& _value)
        {
            codyco::torquebalancing::LockGuard guard(m_lock);
            m_value = _value;
        }

        
    }
}

#endif /* end of include guard: REFERENCE_H */
