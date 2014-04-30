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
        
        class ReferenceGenerator: public ::yarp::os::RateThread
        {
        public:
            ReferenceGenerator(int period, Reference& reference, ReferenceGeneratorInputReader& reader);
            
            virtual bool threadInit();
            virtual void threadRelease();
            virtual void run();
            
        private:
            Reference& m_reference;
            ReferenceGeneratorInputReader& m_reader;
            
            Eigen::VectorXd m_proportionalGains;
            Eigen::VectorXd m_derivativeGains;
            Eigen::VectorXd m_integralGains;
        };
        
        class ReferenceGeneratorInputReader {
        public:
            virtual ~ReferenceGeneratorInputReader();
            virtual Eigen::VectorXd& getSignal() = 0;
            virtual Eigen::VectorXd& getSignalDerivative() = 0;
        };

    }
}


#endif /* end of include guard: REFERENCEGENERATOR_H */
