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

#ifndef CONFIG_H
#define CONFIG_H

#define ACTUATED_DOFS 25
#define TOTAL_DOFS 31

namespace yarp {
    namespace os {
        class Mutex;
    }
}

namespace codyco {
    namespace torquebalancing {
        //cannot use extern on the template.. this is quite strange. To be investigated.
//        extern const int actuatedDOFs; /*!< number of actuated degree of freedom */
//        extern const int totalDOFs; /*!< total number of degree of freedom. For a floating base robot this is usually the number of actuated joints plus 6 fictitious joints*/
        extern const int PseudoInverseTolerance;
        
        //move this class in codyco commons
        /** @class provides a lock guard around a Yarp Mutex.
         * This class implements the same behaviour of std::unique_lock in C++11
         */
        class LockGuard
        {
        public:
            explicit LockGuard(yarp::os::Mutex& mutex);
            ~LockGuard();
            
        private:
            //disable copy constructor and assignment operator
            LockGuard(const LockGuard&);
            LockGuard& operator=(const LockGuard&);
            
            yarp::os::Mutex& m_mutex;
        };
        
    }
}

//namespace Eigen {
//    template<typename _Scalar, int _Rows, int _Cols>
//    class Matrix;
//    
//    typedef Eigen::Matrix<double, TOTAL_DOFS, 1> JointVector;
//    typedef Eigen::Matrix<double, ACTUATED_DOFS, 1> ActuatedVector;
//}


#endif //CONFIG_H
