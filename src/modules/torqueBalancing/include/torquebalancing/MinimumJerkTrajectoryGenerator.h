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

#ifndef MINIMUMJERKTRAJECTORYGENERATOR_H
#define MINIMUMJERKTRAJECTORYGENERATOR_H

#include "ReferenceGenerator.h"
#include <Eigen/Core>


namespace iCub {
    namespace ctrl {
        class minJerkTrajGen;
    }
}

namespace yarp {
    namespace sig {
        class Vector;
    }
}

namespace codyco {
    namespace torquebalancing {
        
        class MinimumJerkTrajectoryGenerator : public ReferenceFilter {
        public:
            MinimumJerkTrajectoryGenerator(int dimension);
            
            virtual ~MinimumJerkTrajectoryGenerator();
            
            virtual ReferenceFilter* clone() const;
            
            virtual bool initializeTimeParameters(double sampleTime,
                                                  double duration);
            
            virtual bool computeReference(const Eigen::VectorXd& setPoint,
                                          const Eigen::VectorXd& currentValue,
                                          double initialTime = 0.0,
                                          bool initFilter = false);
            
            virtual bool updateTrajectoryForCurrentTime(double currentTime);

            virtual const Eigen::VectorXd& getComputedValue();
            virtual const Eigen::VectorXd& getComputedDerivativeValue();
            virtual const Eigen::VectorXd& getComputedSecondDerivativeValue();

        private:
            
            int m_size;
            iCub::ctrl::minJerkTrajGen* m_minimumJerkGenerator;
            Eigen::VectorXd m_computedPosition;
            Eigen::VectorXd m_computedVelocity;
            Eigen::VectorXd m_computedAcceleration;
            
            double m_sampleTime;
            double m_duration;
            
            yarp::sig::Vector* m_yarpReference;
            yarp::sig::Vector* m_yarpInitialValue;
        };
    }
}

#endif /* end of include guard: MINIMUMJERKTRAJECTORYGENERATOR_H */
