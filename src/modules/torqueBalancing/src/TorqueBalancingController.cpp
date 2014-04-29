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

#include "TorqueBalancingController.h"
#include <wbi/wholeBodyInterface.h>
#include <wbi/wbiUtil.h>


#include <Eigen/LU>
#include <Eigen/SVD>

namespace codyco {
    namespace torquebalancing {
        
        TorqueBalancingController::TorqueBalancingController(int period)
        : RateThread(period){}
        
        TorqueBalancingController::~TorqueBalancingController() {}

#pragma mark - RateThread methods
        bool TorqueBalancingController::threadInit()
        {
            using namespace Eigen;
            //Initialize constant variables
            //centroidal force matrix
            m_centroidalForceMatrix.setZero();
            m_centroidalForceMatrix.block<3, 3>(0, 0) = Matrix3d::Identity();
            m_centroidalForceMatrix.block<3, 3>(0, 6) = Matrix3d::Identity();
            m_centroidalForceMatrix.block<3, 3>(3, 3) = Matrix3d::Identity();
            m_centroidalForceMatrix.block<3, 3>(3, 9) = Matrix3d::Identity();
            //gravity
            m_gravityForce.setZero();
            return true;
        }
        
        void TorqueBalancingController::threadRelease()
        {
            
        }
        
        void TorqueBalancingController::run()
        {
            //read references
            
            //read / update state
            updateRobotState();
            
            //compute desired feet forces
            computeFeetForces(m_desiredCOMAcceleration, m_desiredFeetForces);
            
            //compute torques
            computeTorques(m_desiredFeetForces, m_torques);
            
            //write torques
            
            
        }
     
        bool TorqueBalancingController::updateRobotState()
        {
            //read positions and velocities
            m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointPositions.data());
            m_robot->getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointVelocities.data());
            
            //update mass matrix
            m_robot->computeMassMatrix(m_jointPositions.data(), m_worldFrame, m_massMatrix.data());
            m_robot->computeCentroidalMomentum(m_jointPositions.data(), m_worldFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_centroidalMomentum.data());

            m_robot->forwardKinematics(m_jointPositions.data(), m_worldFrame, m_centerOfMassLinkID, m_rotoTranslationVector.data());
            m_centerOfMassPosition = m_rotoTranslationVector.head<3>();
            m_robot->forwardKinematics(m_jointPositions.data(), m_worldFrame, m_leftFootLinkID, m_leftFootPosition.data());
            m_robot->forwardKinematics(m_jointPositions.data(), m_worldFrame, m_rightFootLinkID, m_rightFootPosition.data());
            
            return true;
        }
        
        void TorqueBalancingController::computeFeetForces(const Eigen::Matrix<double, 3, 1>& desiredCOMAcceleration, Eigen::Matrix<double, 12, 1>& desiredFeetForces)
        {
            using namespace Eigen;
            double mass = m_massMatrix(0, 0);
            m_gravityForce(2) = -mass * 9.81;
            
            //building centroidalForceMatrix
            skewSymmentricMatrix(m_leftFootPosition.head<3>() - m_centerOfMassPosition, m_3DMatrix);
            m_centroidalForceMatrix.block<3, 3>(3, 0) = m_3DMatrix;
            skewSymmentricMatrix(m_rightFootPosition.head<3>() - m_centerOfMassPosition, m_3DMatrix);
            m_centroidalForceMatrix.block<3, 3>(3, 6) = m_3DMatrix;
            
            m_desiredCentroidalMomentum.head<3>() = mass * desiredCOMAcceleration;
            m_desiredCentroidalMomentum.tail<3>() = -m_centroidalMomentumGain * m_centroidalMomentum.tail<3>();

            desiredFeetForces = m_centroidalForceMatrix.jacobiSvd(ComputeThinU | ComputeThinV).solve(m_desiredCentroidalMomentum - m_gravityForce);
            
        }
        
        void TorqueBalancingController::computeTorques(const Eigen::Matrix<double, 12, 1>& desiredFeetForces, Eigen::Matrix<double, ACTUATED_DOFS, 1>& torques)
        {
            using namespace Eigen;
            //update mass matrix
//            m_inverseBaseMassMatrix = m_massMatrix.topLeftCorner<6, 6>().inverse().eval();
//            m_massMatrixSchurComplement = m_massMatrix.block<ACTUATED_DOFS, ACTUATED_DOFS>(6, 6) - m_massMatrix.block<ACTUATED_DOFS, 6>(6, 0) * m_inverseBaseMassMatrix * m_massMatrix.block<6, ACTUATED_DOFS>(0, 6);
//            
//            //update jacobians (both feet in one variable)
//            m_robot->computeJacobian(m_jointPositions.data(), m_worldFrame, m_leftFootLinkID, m_feetJacobian.data());
//            m_robot->computeJacobian(m_jointPositions.data(), m_worldFrame, m_rightFootLinkID, m_feetJacobian.data() + (sizeof(double) * 6 * TOTAL_DOFS));
//            
//            m_robot->computeDJdq(m_jointPositions.data(), m_worldFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_leftFootLinkID, m_feetDJacobianDq.data());
//            m_robot->computeDJdq(m_jointPositions.data(), m_worldFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_rightFootLinkID, m_feetDJacobianDq.data() + (sizeof(double) * 6 * TOTAL_DOFS));
//            
//            //update auxiliary variables
//            //SBar(1:6, :) = - M(1:6, 1:6)^-1 * M(1:6, 7:end)
//            m_SBar.block<6, ACTUATED_DOFS>(0, 0) = -m_inverseBaseMassMatrix * m_massMatrix.block<6, ACTUATED_DOFS>(0, 6);
//
//            m_feetJacobianTimesSBar = m_feetJacobian * m_SBar;
//            pseudoInverse(m_feetJacobianTimesSBar, PseudoInverseTolerance, m_feetJacobianTimesSBarPseudoInverse);
//            
//            m_robot->computeGeneralizedBiasForces(m_jointPositions.data(), m_worldFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_gravityUnitVector, m_generalizedBiasForces.data());
//
//            m_desiredJointAcceleration2 = m_feetJacobianTimesSBarPseudoInverse * (m_feetJacobian * m_UMatrix.transpose() * m_inverseBaseMassMatrix * (m_generalizedBiasForces.block<6, 1>(0,0) - m_feetJacobian.transpose() * desiredFeetForces) - m_feetDJacobianDq);
//            //TODO: wait to complete this function for a proper test on matlab
//            
//            pseudoInverse(m_SBar * (Matrix<double, ACTUATED_DOFS, ACTUATED_DOFS>::Identity() - m_feetJacobianTimesSBarPseudoInverse * m_feetJacobianTimesSBar), PseudoInverseTolerance, m_task2PseudoInverse);
//            m_desiredJointAcceleration1 = m_desiredJointAcceleration2 + m_task2PseudoInverse * (q_star - Ji * m_SBar * m_desiredJointAcceleration2);
//            
//            torques = -m_feetJacobianTimesSBar.transpose() * desiredFeetForces + m_massMatrixSchurComplement * m_desiredJointAcceleration1 + m_SBar.transpose() * [h(1:6); h(7:end)];
            
            
//            
//            function tau  = computeTau(fStar, qDDstar, M, h, Jc, dJc_dq, index)
//            %#codegen
//            
//            PINV_TOL = 1e-5;
//            g        = 9.81;
//            
//            Mb    = M(1:6,1:6);
//            MbInv = inv(Mb);
//            Mbj   = M(1:6,7:end);
//            Mj    = M(7:end,7:end);
//            U     = [eye(6) , zeros(6,25)];
//            Jcb   = Jc*U';
//            SBar  = [  -MbInv*Mbj
//                     eye(25)      ];
//            NjInv = Mj - Mbj'*MbInv*Mbj;
//            
//            JcSBar     = Jc*SBar;
//            JcSBarPinv = pinv(JcSBar, PINV_TOL);
//
//                
//                Ji     = [zeros(25,6), eye(25)];
//            Ji     = Ji(index,:);
//            q_star = qDDstar(index);
//            
//            d2q_2 = JcSBarPinv*(Jcb*MbInv*(h(1:6) - Jcb'*fStar) - dJc_dq);
//            N_p1  = eye(25) - JcSBarPinv * JcSBar;
//            d2q_1 = d2q_2 + pinv(Ji*SBar*N_p1, PINV_TOL)*(q_star - Ji * SBar * d2q_2);
//                                           
//            tau = - JcSBar'*fStar + NjInv*d2q_1 + SBar'*[h(1:6);h(7:end)];
            

        }
        
#pragma mark - Auxiliary functions
        
        //return value should be optimized by compiler RVO
        void TorqueBalancingController::skewSymmentricMatrix(const Eigen::Vector3d& vector, Eigen::Matrix3d& skewSymmetricMatrix)
        {
            skewSymmetricMatrix.setZero();
//            S = [   0,   -w(3),    w(2);
//                 w(3),   0,     -w(1);
//                 -w(2),  w(1),     0   ];
            skewSymmetricMatrix(0, 1) = -vector(2);
            skewSymmetricMatrix(0, 2) = vector(1);
            skewSymmetricMatrix(1, 2) = -vector(0);
            skewSymmetricMatrix.bottomLeftCorner<2, 2>() = -skewSymmetricMatrix.topRightCorner<2, 2>().transpose();
        }
        
        template <typename Derived1, typename Derived2>
        void TorqueBalancingController::pseudoInverse(const Eigen::MatrixBase<Derived1>& A,
                           double tolerance,
                           Eigen::MatrixBase<Derived2>& Apinv)
        {
            using namespace Eigen;
            JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
            typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType singularValues = svd.singularValues();
            //I don't get why I cannot use the singular vector as in http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
            MatrixXd singularInverse = singularValues.asDiagonal();

            for (int idx = 0; idx < singularValues.size(); idx++) {
                singularInverse(idx, idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
            }

//            for (int idx = 0; idx < singularValues.size(); idx++) {
//                singularValues(idx) = tolerance > 0 && singularValues(idx) > tolerance ? 1.0 / singularValues(idx) : 0.0;
//            }
//            auto var = svd.matrixV() * sigmaDamped;
//            auto var2 = sigmaDamped * svd.matrixU().transpose();
//            
//            Apinv = var * svd.matrixU().transpose();
            
            Apinv = (svd.matrixV() * singularInverse * svd.matrixU().adjoint());   //pseudoinverse
        }
        
    }
}
