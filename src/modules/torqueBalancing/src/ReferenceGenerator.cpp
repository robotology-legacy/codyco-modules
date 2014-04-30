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

#include "ReferenceGenerator.h"
#include "Reference.h"
#include <wbi/wholeBodyInterface.h>
#include <yarp/os/Time.h>

namespace codyco {
    namespace torquebalancing {
        
//TODO: add min jerk ?
#pragma mark - ReferenceGenerator methods
        
        ReferenceGenerator::ReferenceGenerator(int period, Reference& reference, ReferenceGeneratorInputReader& reader)
        : RateThread(period)
        , m_outputReference(reference)
        , m_reader(reader)
        , m_computedReference(reference.valueSize())
        , m_integralTerm(reader.signalSize())
        , m_proportionalGains(reader.signalSize())
        , m_derivativeGains(reader.signalSize())
        , m_integralGains(reader.signalSize())
        , m_signalReference(reference.valueSize())
        , m_signalDerivativeReference(reference.valueSize())
        , m_signalFeedForward(reference.valueSize())
        , m_previousTime(-1)
        , m_active(false) {}

        bool ReferenceGenerator::threadInit()
        {
            m_proportionalGains.setZero();
            m_derivativeGains.setZero();
            m_integralGains.setZero();
            
            m_computedReference.setZero();
            m_integralTerm.setZero();
            
            m_signalReference.setZero();
            m_signalDerivativeReference.setZero();
            m_signalFeedForward.setZero();
            
            //avoid garbage in the generated reference
            m_outputReference.setValue(m_computedReference);
            m_outputReference.setValid(false);
            
            return true;
        }
        
        void ReferenceGenerator::threadRelease()
        {
            
        }
        
        void ReferenceGenerator::run()
        {
            if (isActiveState()) {
                double now = yarp::os::Time::now();
                if (m_previousTime < 0) m_previousTime = now;
                double dt = now - m_previousTime;
                
                //compute pid
                Eigen::VectorXd error = m_signalReference - m_reader.getSignal();
                
                //TODO: Add integral limits
                m_integralTerm += dt * error;
                
                m_computedReference = m_signalFeedForward
                + m_proportionalGains.asDiagonal() * error
                + m_derivativeGains.asDiagonal() * (m_signalDerivativeReference - m_reader.getSignalDerivative())
                + m_integralGains.asDiagonal() * m_integralTerm;
                m_outputReference.setValue(m_computedReference);
                
                m_previousTime = now;
            }
        }
        
        Eigen::VectorXd& ReferenceGenerator::signalReference()
        {
            return m_signalReference;
        }
        
        void ReferenceGenerator::setSignalReference(Eigen::VectorXd& reference)
        {
            m_signalReference = reference;
        }
        
        Eigen::VectorXd ReferenceGenerator::signalDerivativeReference()
        {
            return m_signalDerivativeReference;
        }
        
        void ReferenceGenerator::setSignalDerivativeReference(Eigen::VectorXd& derivativeReference)
        {
            m_signalDerivativeReference = derivativeReference;
        }
        
        Eigen::VectorXd& ReferenceGenerator::signalFeedForward()
        {
            return m_signalFeedForward;
        }
        
        void ReferenceGenerator::setSignalFeedForward(Eigen::VectorXd& feedforward)
        {
            m_signalFeedForward = feedforward;
        }
        
        void ReferenceGenerator::setActiveState(bool isActive)
        {
            if (m_active == isActive) return;
            if (isActive) {
                //reset integral state
                m_integralTerm.setZero();
                m_previousTime = -1;
                
            } else {
                m_outputReference.setValid(false);
            }
            m_active = isActive;
        }
        
        bool ReferenceGenerator::isActiveState()
        {
            return m_active;
        }
        
#pragma mark - ReferenceGeneratorInputReader methods
        ReferenceGeneratorInputReader::~ReferenceGeneratorInputReader() {}
        
    }
}