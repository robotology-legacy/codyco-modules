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
            
            Eigen::VectorXd& signalReference();
            void setSignalReference(Eigen::VectorXd& reference);
            
            Eigen::VectorXd signalDerivativeReference();
            void setSignalDerivativeReference(Eigen::VectorXd& derivativeReference);
            
            Eigen::VectorXd& signalFeedForward();
            void setSignalFeedForward(Eigen::VectorXd& reference);
            
            void setActiveState(bool isActive);
            bool isActiveState();
            
        private:
            Reference& m_outputReference;
            ReferenceGeneratorInputReader& m_reader;
            
            Eigen::VectorXd m_computedReference;
            Eigen::VectorXd m_integralTerm;
            
            Eigen::VectorXd m_proportionalGains;
            Eigen::VectorXd m_derivativeGains;
            Eigen::VectorXd m_integralGains;
            
            Eigen::VectorXd m_signalReference;
            Eigen::VectorXd m_signalDerivativeReference;
            Eigen::VectorXd m_signalFeedForward;
            
            double m_previousTime;
            bool m_active;
        };
        
        class ReferenceGeneratorInputReader {
        public:
            virtual ~ReferenceGeneratorInputReader();
            virtual Eigen::VectorXd& getSignal() = 0;
            virtual Eigen::VectorXd& getSignalDerivative() = 0;
            virtual int signalSize() const = 0;
        };

    }
}


#endif /* end of include guard: REFERENCEGENERATOR_H */
