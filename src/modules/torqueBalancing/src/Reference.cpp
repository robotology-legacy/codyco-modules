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

#include "Reference.h"
#include "config.h"
#include <codyco/LockGuard.h>


namespace codyco {
    namespace torquebalancing {

#pragma mark - Reference implementation
        
        Reference::Reference(int referenceSize)
        : m_value(referenceSize)
        , m_valid(false)
        , m_valueSize(referenceSize) {}
        
        Reference::~Reference() {}
        
        const Eigen::VectorXd& Reference::value()
        {
            codyco::LockGuard guard(m_lock);
            return m_value;
        }
        
        void Reference::setValue(Eigen::VectorXd& _value)
        {
            codyco::LockGuard guard(m_lock);
            m_value = _value;
            m_valid = true;
        }
        
        void Reference::setValid(bool isValid)
        {
            codyco::LockGuard guard(m_lock);
            m_valid = isValid;
        }
        
        bool Reference::isValid()
        {
            codyco::LockGuard guard(m_lock);
            return m_valid;
        }
        
        int Reference::valueSize() const
        {
            return m_valueSize;
        }
        
#pragma mark - ControllerReferences implementation
        
        ControllerReferences::ControllerReferences()
        : m_desiredCOMAcceleration(3)
        , m_desiredHandsPosition(14)
        , m_desiredHandsForce(12)
        {}
        
        Reference& ControllerReferences::desiredCOMAcceleration()
        {
            return m_desiredCOMAcceleration;
        }
        
        Reference& ControllerReferences::desiredHandsPosition()
        {
            return m_desiredHandsPosition;
        }
        
        Reference& ControllerReferences::desiredHandsForce()
        {
            return m_desiredHandsForce;
        }
    }
}
