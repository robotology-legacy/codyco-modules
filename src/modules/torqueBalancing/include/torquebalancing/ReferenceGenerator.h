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

#ifndef REFERENCEGENERATOR_H
#define REFERENCEGENERATOR_H

#include <yarp/os/RateThread.h>
#include <Eigen/Core>
#include <cmath>
#include <yarp/os/Mutex.h>

namespace wbi {
    class wholeBodyIterface;
}

namespace codyco {
    namespace torquebalancing {
        
        class ReferenceGeneratorInputReader;
        class Reference;
        
        /** This class is responsible of generating a proper reference signal.
         *
         * This class is agnostic of the reference type and dimension.
         * It generates a reference signal as the output of a PID (min jerk trajectory also in the future?). It reads the current value for the signal and signal derivatives from a class which implements the ReferenceGeneratorInputReader protocol.
         * It then writes the computed reference to the reference object.
         *
         */
        class ReferenceGenerator: public ::yarp::os::RateThread
        {
        public:
            /** Constructor.
             *
             * @param period thread period
             * @param reference object in which to save the computed reference trajectory
             * @param reader object implementing the ReferenceGeneratorInputReader protocol used to obtain the current state of the system.
             */
            ReferenceGenerator(int period, Reference& reference, ReferenceGeneratorInputReader& reader);
            
            virtual bool threadInit();
            virtual void threadRelease();
            virtual void run();
            
#pragma mark - Getter and setter
            
            const Eigen::VectorXd& signalReference();
            void setSignalReference(const Eigen::VectorXd& reference);
            
            const Eigen::VectorXd signalDerivativeReference();
            void setSignalDerivativeReference(const Eigen::VectorXd& derivativeReference);
            
            const Eigen::VectorXd& signalFeedForward();
            void setSignalFeedForward(const Eigen::VectorXd& feedforward);
            
            /** Sets all the controller references at once.
             *
             * Use this method if you need to set all the references of the controller.
             * @param reference the reference of the signal
             * @param derivativeReference the reference of the derivative of the signal
             * @param feedforward the feedforward term
             */
            void setAllReferences(const Eigen::VectorXd& reference,
                                  const Eigen::VectorXd& derivativeReference,
                                  const Eigen::VectorXd& feedforward);
            
            /** Sets the state of the controller.
             *
             * If the state is active the controller start to compute and to update the trajectory. If false the controller does nothing.
             * If the controller is already in the state requested as input the command is ignored.
             * @param isActive the new state of the controller
             */
            void setActiveState(bool isActive);
            bool isActiveState();
            
            const Eigen::VectorXd& proportionalGains();
            void setProportionalGains(const Eigen::VectorXd& proportionalGains);
            
            const Eigen::VectorXd& derivativeGains();
            void setDerivativeGains(const Eigen::VectorXd& derivativeGains);
            
            const Eigen::VectorXd& integralGains();
            void setIntegralGains(const Eigen::VectorXd& integralGains);
            
            double integralLimit();
            void setIntegralLimit(double integralLimit);
            
            /** Sets all gain for the PID controller.
             *
             * This function sets all the gains of the controller in one shot.
             * If the new integral limit is NAN (C99 cmath constant) then it is ignored. Pass std::numeric_limits<double>::max() to disable integral limit.
             * @param proportionalGains the new proportional gains
             * @param derivativeGains the new derivative gains
             * @param integralGains the new integral gains.
             * @param integralLimit the new integral limit or NAN to ignore it
             */
            void setAllGains(const Eigen::VectorXd& proportionalGains,
                             const Eigen::VectorXd& derivativeGains,
                             const Eigen::VectorXd& integralGains,
                             double integralLimit = NAN);
            
        private:
            
            void limitIntegral(const Eigen::Ref<Eigen::VectorXd>& integral, Eigen::Ref<Eigen::VectorXd> limitedIntegral);
            
            Reference& m_outputReference;
            ReferenceGeneratorInputReader& m_reader;
            
            Eigen::VectorXd m_computedReference;
            Eigen::VectorXd m_integralTerm;
            
            Eigen::VectorXd m_proportionalGains;
            Eigen::VectorXd m_derivativeGains;
            Eigen::VectorXd m_integralGains;
            
            double m_integralLimit;
            
            Eigen::VectorXd m_signalReference;
            Eigen::VectorXd m_signalDerivativeReference;
            Eigen::VectorXd m_signalFeedForward;
            
            double m_previousTime;
            bool m_active;
            
            yarp::os::Mutex m_mutex;
        };
        
        class ReferenceGeneratorInputReader {
        public:
            virtual ~ReferenceGeneratorInputReader();
            virtual const Eigen::VectorXd& getSignal() = 0;
            virtual const Eigen::VectorXd& getSignalDerivative() = 0;
            virtual int signalSize() const = 0;
        };

    }
}


#endif /* end of include guard: REFERENCEGENERATOR_H */
