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
#include <codyco/LockGuard.h>
#include <yarp/os/Time.h>
#include <limits>

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
        , m_integralLimit(std::numeric_limits<double>::max())
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
            m_computedReference.setZero();
            m_outputReference.setValue(m_computedReference);
            m_outputReference.setValid(false);
        }
        
        void ReferenceGenerator::run()
        {
            codyco::LockGuard guard(m_mutex);
            if (m_active) {
                double now = yarp::os::Time::now();
                if (m_previousTime < 0) m_previousTime = now;
                double dt = now - m_previousTime;
                
                //compute pid
                Eigen::VectorXd error = m_signalReference - m_reader.getSignal();
                
                m_integralTerm += dt * error;
                limitIntegral(m_integralTerm, m_integralTerm);
                
                m_computedReference = m_signalFeedForward
                + m_proportionalGains.asDiagonal() * error
                + m_derivativeGains.asDiagonal() * (m_signalDerivativeReference - m_reader.getSignalDerivative())
                + m_integralGains.asDiagonal() * m_integralTerm;
                m_outputReference.setValue(m_computedReference);
                
                m_previousTime = now;
            }
        }
        
        const Eigen::VectorXd& ReferenceGenerator::signalReference()
        {
            codyco::LockGuard guard(m_mutex);
            return m_signalReference;
        }
        
        void ReferenceGenerator::setSignalReference(const Eigen::VectorXd& reference)
        {
            codyco::LockGuard guard(m_mutex);
            m_signalReference = reference;
        }
        
        const Eigen::VectorXd ReferenceGenerator::signalDerivativeReference()
        {
            codyco::LockGuard guard(m_mutex);
            return m_signalDerivativeReference;
        }
        
        void ReferenceGenerator::setSignalDerivativeReference(const Eigen::VectorXd& derivativeReference)
        {
            codyco::LockGuard guard(m_mutex);
            m_signalDerivativeReference = derivativeReference;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::signalFeedForward()
        {
            codyco::LockGuard guard(m_mutex);
            return m_signalFeedForward;
        }
        
        void ReferenceGenerator::setSignalFeedForward(const Eigen::VectorXd& feedforward)
        {
            codyco::LockGuard guard(m_mutex);
            m_signalFeedForward = feedforward;
        }
        
        void ReferenceGenerator::setAllReferences(const Eigen::VectorXd& reference,
                                                  const Eigen::VectorXd& derivativeReference,
                                                  const Eigen::VectorXd& feedforward)
        {
            codyco::LockGuard guard(m_mutex);
            m_signalReference = reference;
            m_signalDerivativeReference = derivativeReference;
            m_signalFeedForward = feedforward;
        }
        
        void ReferenceGenerator::setActiveState(bool isActive)
        {
            codyco::LockGuard guard(m_mutex);
            if (m_active == isActive) return;
            if (isActive) {
                //reset integral state
                m_integralTerm.setZero();
                m_previousTime = -1;
                //reset references
                m_signalReference = m_reader.getSignal();
                m_signalDerivativeReference = m_reader.getSignalDerivative();
                m_signalFeedForward.setZero();
                
            } else {
                m_outputReference.setValid(false);
            }
            m_active = isActive;
        }
        
        bool ReferenceGenerator::isActiveState()
        {
            codyco::LockGuard guard(m_mutex);
            return m_active;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::proportionalGains()
        {
            codyco::LockGuard guard(m_mutex);
            return m_proportionalGains;
        }
        
        void ReferenceGenerator::setProportionalGains(const Eigen::VectorXd& proportionalGains)
        {
            codyco::LockGuard guard(m_mutex);
            m_proportionalGains = proportionalGains;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::derivativeGains()
        {
            codyco::LockGuard guard(m_mutex);
            return m_derivativeGains;
        }
        
        void ReferenceGenerator::setDerivativeGains(const Eigen::VectorXd& derivativeGains)
        {
            codyco::LockGuard guard(m_mutex);
            m_derivativeGains = derivativeGains;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::integralGains()
        {
            codyco::LockGuard guard(m_mutex);
            return m_integralGains;
        }
        
        void ReferenceGenerator::setIntegralGains(const Eigen::VectorXd& integralGains)
        {
            codyco::LockGuard guard(m_mutex);
            m_integralGains = integralGains;
        }
        
        double ReferenceGenerator::integralLimit()
        {
            codyco::LockGuard guard(m_mutex);
            return m_integralLimit;
        }
        
        void ReferenceGenerator::setIntegralLimit(double integralLimit)
        {
            if (!isnan(integralLimit)) {
                codyco::LockGuard guard(m_mutex);
                m_integralLimit = std::abs(integralLimit);
            }
        }
        
        void ReferenceGenerator::limitIntegral(const Eigen::Ref<Eigen::VectorXd>& integral, Eigen::Ref<Eigen::VectorXd> limitedIntegral)
        {
            limitedIntegral = integral.array().cwiseMin(m_integralLimit).cwiseMax(-m_integralLimit).matrix();
        }
        
        void ReferenceGenerator::setAllGains(const Eigen::VectorXd& proportionalGains,
                                             const Eigen::VectorXd& derivativeGains,
                                             const Eigen::VectorXd& integralGains,
                                             double integralLimit)
        {
            codyco::LockGuard guard(m_mutex);
            m_proportionalGains = proportionalGains;
            m_derivativeGains = derivativeGains;
            m_integralGains = integralGains;
            if (!isnan(integralLimit)) {
                m_integralLimit = std::abs(integralLimit);
            }
        }
        
        const Eigen::VectorXd& ReferenceGenerator::computedReference()
        {
            codyco::LockGuard guard(m_mutex);
            return m_computedReference;
        }

        
#pragma mark - ReferenceGeneratorInputReader methods
        
        ReferenceGeneratorInputReader::~ReferenceGeneratorInputReader() {}
        
        bool ReferenceGeneratorInputReader::init() { return true; }
        
    }
}