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

#ifndef TORQUEBALANCINGCONTROLLER_H
#define TORQUEBALANCINGCONTROLLER_H

#include "config.h"
#include <yarp/os/RateThread.h>
#include <wbi/wbiUtil.h>
#include <Eigen/Core>

namespace wbi {
    class wholeBodyInterface;
}

namespace codyco {
    namespace torquebalancing {
        
        /** @brief Represents the actual controller
         *
         */
        class TorqueBalancingController: public yarp::os::RateThread
        {
        public:
            TorqueBalancingController(int period);
            virtual ~TorqueBalancingController();
            
#pragma mark - RateThread methods
            virtual bool threadInit();
            virtual void threadRelease();
            virtual void run();
            
        private:
            
            bool updateRobotState();
            void computeFeetForces(const Eigen::Matrix<double, 3, 1>& desiredCOMAcceleration, Eigen::Matrix<double, 12, 1>& desiredFeetForces);
            void computeTorques(const Eigen::Matrix<double, 12, 1>& desiredFeetForces, Eigen::Matrix<double, ACTUATED_DOFS, 1>& torques);
            
            //return value should be optimized by compiler RVO
            void skewSymmentricMatrix(const Eigen::Vector3d& vector, Eigen::Matrix3d& skewSymmetricMatrix);
            
            wbi::wholeBodyInterface* m_robot;
            wbi::Frame m_worldFrame;
            
            //configuration-time constants
            int m_leftFootLinkID;
            int m_rightFootLinkID;
            int m_centerOfMassLinkID;
            
            //Gains
            double m_centroidalMomentumGain;
            
            //references
            Eigen::Vector3d m_desiredCOMAcceleration;
            Eigen::Matrix<double, 12, 1> m_desiredFeetForces;
            Eigen::Matrix<double, 6, 1> m_desiredCentroidalMomentum;
            
            //state of the robot
            Eigen::Matrix<double, TOTAL_DOFS, 1> m_jointPositions;
            Eigen::Matrix<double, TOTAL_DOFS, 1> m_jointVelocities;
            Eigen::Matrix<double, ACTUATED_DOFS, 1> m_torques;
            Eigen::Matrix<double, 6, 1> m_baseVelocity;
            Eigen::Vector3d m_centerOfMassPosition;
            Eigen::Matrix<double, 7, 1> m_rightFootPosition;
            Eigen::Matrix<double, 7, 1> m_leftFootPosition;
            
            //Jacobians
            Eigen::Matrix<double, 6 + 6, TOTAL_DOFS> m_feetJacobian;
            Eigen::Matrix<double, 6 + 6, TOTAL_DOFS> m_feetDJacobianDq;
            
            //Kinematic and dynamic variables
            Eigen::Matrix<double, TOTAL_DOFS, TOTAL_DOFS> m_massMatrix;
            Eigen::Matrix<double, 6, 6> m_inverseBaseMassMatrix;
            Eigen::Matrix<double, ACTUATED_DOFS, ACTUATED_DOFS> m_massMatrixSchurComplement;
            Eigen::Matrix<double, TOTAL_DOFS, 1> m_generalizedBiasForces;
            Eigen::Matrix<double, 6, 1> m_centroidalMomentum;
            
            //variables used in computation.
            //variables names (when possible) are taken from the [DelPrete 2013] PhD thesis
            Eigen::Matrix<double, TOTAL_DOFS, ACTUATED_DOFS> m_SBar;
            Eigen::Matrix<double, 12, ACTUATED_DOFS> m_feetJacobianTimesSBar;
            Eigen::Matrix<double, ACTUATED_DOFS, 12> m_feetJacobianTimesSBarPseudoInverse;
            Eigen::Matrix<double, 6, 12> m_centroidalForceMatrix;
            Eigen::Matrix<double, 6, 1> m_gravityForce;

            Eigen::Matrix3d m_3DMatrix;

            //Tasks variables
            Eigen::Matrix<double, ACTUATED_DOFS, 1> m_desiredJointAcceleration2;
            Eigen::Matrix<double, ACTUATED_DOFS, 1> m_desiredJointAcceleration1;
//            Eigen::Matrix<double, ACTUATED_DOFS, something> m_task2PseudoInverse;
            
            
            //constant auxiliary variables
            Eigen::Matrix<double, 6, TOTAL_DOFS> m_UMatrix;
            double m_gravityUnitVector[3];
            Eigen::Matrix<double, 7, 1> m_rotoTranslationVector;
            
            //to be moved in codyco_math
            template <typename Derived1, typename Derived2>
            void pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                               double tolerance,
                               Eigen::MatrixBase<Derived2>& Apinv);
            
        };
    }
}

#endif /* end of include guard: TORQUEBALANCINGCONTROLLER_H */
