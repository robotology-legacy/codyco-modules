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
#include <yarp/os/LockGuard.h>
#include <codyco/MathUtils.h>
#include <yarp/os/Time.h>
#include <limits>

namespace codyco {
    namespace torquebalancing {
        
#pragma mark - ReferenceGenerator methods
        
        ReferenceGenerator::ReferenceGenerator(int period, Reference& reference, ReferenceGeneratorInputReader& reader, const std::string& name)
        : RateThread(period)
        , m_name(name)
        , m_outputReference(reference)
        , m_reader(reader)
        , m_referenceFilter(0)
        , m_computedReference(reference.valueSize())
        , m_integralTerm(reader.signalSize())
        , m_error(reader.signalSize())
        , m_proportionalGains(reader.signalSize())
        , m_derivativeGains(reader.signalSize())
        , m_integralGains(reader.signalSize())
        , m_integralLimit(std::numeric_limits<double>::max())
        , m_signalReference(reference.valueSize())
        , m_signalDerivativeReference(reference.valueSize())
        , m_signalFeedForward(reference.valueSize())
        , m_previousTime(-1)
        , m_active(false)
        , m_currentSignalValue(reference.valueSize())
        , m_actualReference(reference.valueSize())
        {
            m_proportionalGains.setZero();
            m_derivativeGains.setZero();
            m_integralGains.setZero();
            
            m_computedReference.setZero();
            m_integralTerm.setZero();
            
            m_signalReference.setZero();
            m_signalDerivativeReference.setZero();
            m_signalFeedForward.setZero();
            
            m_currentSignalValue.setZero();
            m_error.setZero(); //only for monitoring
            
            //avoid garbage in the generated reference
            m_outputReference.setValue(m_computedReference);
            m_outputReference.setValid(false);
        }
        
        ReferenceGenerator::~ReferenceGenerator()
        {
            if (m_referenceFilter) {
                delete m_referenceFilter;
                m_referenceFilter = 0;
            }
        }

        bool ReferenceGenerator::threadInit()
        {
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
            yarp::os::LockGuard guard(m_mutex);
            if (m_active) {
                double now = yarp::os::Time::now();
                if (m_previousTime < 0) m_previousTime = now;
                double dt = now - m_previousTime;

                m_actualReference = m_signalReference;
                if (m_referenceFilter) {
                    m_actualReference = m_referenceFilter->getValueForCurrentTime(now);
                }
                long context = now * 1000; //i use the time in ms as a context
                
                m_currentSignalValue = m_reader.getSignal(context);
                //compute pid
                m_error = m_currentSignalValue - m_actualReference;
                m_integralTerm += dt * m_error;
                limitIntegral(m_integralTerm, m_integralTerm);
                
                m_computedReference = m_signalFeedForward
                - m_proportionalGains.asDiagonal() * m_error
                - m_derivativeGains.asDiagonal() * (m_reader.getSignalDerivative(context) - m_signalDerivativeReference)
                - m_integralGains.asDiagonal() * m_integralTerm;
                m_outputReference.setValue(m_computedReference);

//                if (strcmp("com pid", m_name.c_str()) == 0) {
//                    std::cout << m_actualReference.transpose() << "\n";
//                }

                m_previousTime = now;
            }
        }
        
#pragma mark - Getter and setter
        
        const std::string& ReferenceGenerator::name() const
        {
            return m_name;
        }
        
        void ReferenceGenerator::setReferenceFilter(ReferenceFilter* referenceFilter)
        {
            if (this->isRunning()) return;
            if (m_referenceFilter) {
                delete m_referenceFilter;
                m_referenceFilter = 0;
            }
            if (referenceFilter)
                m_referenceFilter = referenceFilter->clone();
        }
        
        const ReferenceFilter* ReferenceGenerator::referenceFilter()
        {
            return m_referenceFilter;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::signalReference()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_signalReference;
        }
        
        void ReferenceGenerator::setSignalReference(const Eigen::VectorXd& reference)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_signalReference = reference;
            if (m_referenceFilter) {
                //???: If thread is active this is the last updated signal value, otherwise I don't care. The computation will be redone after start
                m_referenceFilter->computeReference(m_signalReference, m_currentSignalValue, m_previousTime);
            }
        }
        
        const Eigen::VectorXd ReferenceGenerator::signalDerivativeReference()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_signalDerivativeReference;
        }
        
        void ReferenceGenerator::setSignalDerivativeReference(const Eigen::VectorXd& derivativeReference)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_signalDerivativeReference = derivativeReference;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::signalFeedForward()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_signalFeedForward;
        }
        
        void ReferenceGenerator::setSignalFeedForward(const Eigen::VectorXd& feedforward)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_signalFeedForward = feedforward;
        }
        
        void ReferenceGenerator::setAllReferences(const Eigen::VectorXd& reference,
                                                  const Eigen::VectorXd& derivativeReference,
                                                  const Eigen::VectorXd& feedforward)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_signalReference = reference;
            if (m_referenceFilter) {
                m_referenceFilter->computeReference(m_signalReference, m_currentSignalValue, m_previousTime);
            }
            m_signalDerivativeReference = derivativeReference;
            m_signalFeedForward = feedforward;
        }
        
        void ReferenceGenerator::setActiveState(bool isActive)
        {
            yarp::os::LockGuard guard(m_mutex);
            if (m_active == isActive) return;
            if (isActive) {
                //reset integral state
                m_integralTerm.setZero();
                m_previousTime = -1;
                //reset references
                m_currentSignalValue = m_reader.getSignal();
                m_signalReference = m_currentSignalValue;
                m_signalDerivativeReference = m_reader.getSignalDerivative();
                m_signalFeedForward.setZero();
                if (m_referenceFilter) {
                    m_referenceFilter->computeReference(m_signalReference, m_currentSignalValue, yarp::os::Time::now(), true);
                }
                
            } else {
                m_outputReference.setValid(false);
            }
            m_active = isActive;
        }
        
        bool ReferenceGenerator::isActiveState()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_active;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::proportionalGains()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_proportionalGains;
        }
        
        void ReferenceGenerator::setProportionalGains(const Eigen::VectorXd& proportionalGains)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_proportionalGains = proportionalGains;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::derivativeGains()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_derivativeGains;
        }
        
        void ReferenceGenerator::setDerivativeGains(const Eigen::VectorXd& derivativeGains)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_derivativeGains = derivativeGains;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::integralGains()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_integralGains;
        }
        
        void ReferenceGenerator::setIntegralGains(const Eigen::VectorXd& integralGains)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_integralGains = integralGains;
        }
        
        double ReferenceGenerator::integralLimit()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_integralLimit;
        }
        
        void ReferenceGenerator::setIntegralLimit(double integralLimit)
        {
            if (!codyco::math::isnan(integralLimit)) {
                yarp::os::LockGuard guard(m_mutex);
                m_integralLimit = std::abs(integralLimit);
            }
        }
        
        void ReferenceGenerator::limitIntegral(const Eigen::Ref<Eigen::VectorXd>& integral, Eigen::Ref<Eigen::VectorXd> limitedIntegral)
        {
//            limitedIntegral = integral.array().cwiseMin(m_integralLimit).cwiseMax(-m_integralLimit).matrix();
            for (int i = 0; i < integral.rows(); i++) {
                limitedIntegral(i) = integral(i) > m_integralLimit ? m_integralLimit
                : (integral(i) < -m_integralLimit ? -m_integralLimit : integral(i));
            }
        }
        
        void ReferenceGenerator::setAllGains(const Eigen::VectorXd& proportionalGains,
                                             const Eigen::VectorXd& derivativeGains,
                                             const Eigen::VectorXd& integralGains,
                                             double integralLimit)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_proportionalGains = proportionalGains;
            m_derivativeGains = derivativeGains;
            m_integralGains = integralGains;
            if (!codyco::math::isnan(integralLimit)) {
                m_integralLimit = std::abs(integralLimit);
            }
        }
        
        const Eigen::VectorXd& ReferenceGenerator::computedReference()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_computedReference;
        }
        
        const Eigen::VectorXd& ReferenceGenerator::instantaneousError()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_error;   
        }
        
        const Eigen::VectorXd& ReferenceGenerator::errorIntegral()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_integralTerm;
        }
        
#pragma mark - ReferenceGeneratorInputReader methods
        
        ReferenceGeneratorInputReader::~ReferenceGeneratorInputReader() {}
        
        bool ReferenceGeneratorInputReader::init() { return true; }
        
#pragma mark - ReferenceFilter methods

        ReferenceFilter::~ReferenceFilter() {}
        
    }
}
