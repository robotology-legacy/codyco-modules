/**
 * Copyright (C) 2015 CoDyCo - Robotics, Brain and Cognitive Sciences Department
 * Italian Institute of Technology
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

#ifndef DYNAMICCONSTRAINT_H
#define DYNAMICCONSTRAINT_H

namespace codyco {
    namespace torquebalancing {

        //TODO: this class should hold the information about
        // - link/frame name
        // - jacobians

        class DynamicConstraint {
        private:
            void * m_implementation;

        public:
            DynamicConstraint();
            ~DynamicConstraint();
            DynamicConstraint(const DynamicConstraint&);
            DynamicConstraint& operator=(const DynamicConstraint&);

            bool init(bool isConstraintActiveAtInit, double timeStep, double transitionTime);

            bool isActive() const;
            bool isActiveWithThreshold(double threshold) const;
            double continuousValue() const;

            void updateStateInterpolation();
            void activate();
            void deactivate();
            
        };
        
    }
}
#endif /* end of include guard: DYNAMICCONSTRAINT_H */
