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
        
        //TODO: maybe another name is better, as it is a quite generic class
        /** This class wraps the concept of a generic vector value.
         * The size is passed at construction and cannot be changed. It can be obtained by calling valueSize() function.
         * The content of this object which is not valid is not guaranteed to contain meaningful values (i.e. it can be garbage, so do not use it).
         * This class is thread-safe for setting and reading values.
         */
        class Reference
        {
        public:
            explicit Reference(int referenceSize);
            ~Reference();
            
            /** Return the current value.
             * Before using the value is some computation check if it is valid or not.
             * @return the current value
             */
            Eigen::VectorXd& value();
            
            /** Sets the value for the current reference.
             * The state of the reference automatically switch to active.
             *
             * @param _value value of the new reference to be saved.
             */
            void setValue(Eigen::VectorXd& _value);
            
            /** Set the valid state of the reference explicitly
             *
             * @param isValid true if the reference value is not garbage. False otherwise
             */
            void setValid(bool isValid);
            
            /** returns the state of the reference.
             * If a reference is not valid its value is not guarantee to be something meaningful
             * @return if the reference is valid or not.
             */
            bool isValid();
            
            /** returns the size of the currently hold value, i.e. the size of the vector returned by value().
             *
             * @return the size of the value
             */
            int valueSize() const;
            
        private:
            yarp::os::Mutex m_lock;
            Eigen::VectorXd m_value;
            bool m_valid;
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
