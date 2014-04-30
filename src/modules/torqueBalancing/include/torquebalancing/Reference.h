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
        
        class Reference
        {
        public:
            explicit Reference(int referenceSize);
            ~Reference();
            
            Eigen::VectorXd& value();
            void setValue(Eigen::VectorXd& _value);
            
            const int valueSize() const;
            
        private:
            yarp::os::Mutex m_lock;
            Eigen::VectorXd m_value;
            const int m_valueSize;
        };
        
        class ControllerReferences
        {
        public:
            ControllerReferences();
            
            /** @brief returns the desired COM acceleration 3-dim reference.
             * @return desired COM acceleration (3 dim)
             */
            Reference& desiredCOMAcceleration();
            
            /** @brief returns the desired Hands positions reference.
             * The vector is a 14 dimension vector, 7 (position and angle-axis representation) for each hand.
             * @return desired desired Hands positions (14 dim)
             */
            Reference& desiredHandsPosition();
            
            /** @brief returns the desired Hands force reference.
             * The vector is a 12 dimension vector, 6 for each hand.
             * @return desired Hands force (12 dim)
             */
            Reference& desiredHandsForce();
        private:
            Reference m_desiredCOMAcceleration;
            Reference m_desiredHandsPosition;
            Reference m_desiredHandsForce;
        };
        
    }
}

#endif /* end of include guard: REFERENCE_H */
