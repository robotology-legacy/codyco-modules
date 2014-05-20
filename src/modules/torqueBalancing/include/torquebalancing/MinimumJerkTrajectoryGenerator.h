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
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#endif
#include <Eigen/Core>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif


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
            
            MinimumJerkTrajectoryGenerator(int dimension);
            
            virtual ~MinimumJerkTrajectoryGenerator();
            
            virtual ReferenceFilter* clone() const;
            
            virtual bool initializeTimeParameters(double sampleTime,
                                                  double duration);
            
            virtual bool computeReference(const Eigen::VectorXd& setPoint,
                                          const Eigen::VectorXd& currentValue,
                                          double initialTime = 0.0);
            
            virtual Eigen::VectorXd getValueForCurrentTime(double currentTime);

        private:
            
            int m_size;
            iCub::ctrl::minJerkTrajGen* m_minimumJerkGenerator;
            Eigen::VectorXd m_computedPosition;
            
            double m_sampleTime;
            double m_duration;
            
            yarp::sig::Vector* m_yarpReference;
            yarp::sig::Vector* m_yarpInitialValue;
        };
    }
}

#endif /* end of include guard: MINIMUMJERKTRAJECTORYGENERATOR_H */
