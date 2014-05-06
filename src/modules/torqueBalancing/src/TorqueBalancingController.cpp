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
#include "Reference.h"
#include <wbi/wholeBodyInterface.h>
#include <wbi/wbiUtil.h>
#include <codyco/MathUtils.h>
#include <codyco/LockGuard.h>


#include <Eigen/LU>
#include <Eigen/SVD>

namespace codyco {
    namespace torquebalancing {
        
        TorqueBalancingController::TorqueBalancingController(int period, ControllerReferences& references, wbi::wholeBodyInterface& robot)
        : RateThread(period)
        , m_robot(robot)
        , m_active(false)
        , m_references(references)
        , m_centroidalMomentumGain(0)
        , m_desiredCOMAcceleration(3)
        , m_desiredFeetForces(12)
        , m_desiredCentroidalMomentum(6)
        , m_jointPositions(totalDOFs)
        , m_jointVelocities(totalDOFs)
        , m_torques(actuatedDOFs)
        , m_baseVelocity(6)
        , m_centerOfMassPosition(3)
        , m_rightFootPosition(7)
        , m_leftFootPosition(7)
        , m_feetJacobian(6 + 6, totalDOFs)
        , m_feetDJacobianDq(12, totalDOFs)
        , m_massMatrix(totalDOFs, totalDOFs)
        , m_inverseBaseMassMatrix(6, 6)
        , m_massMatrixSchurComplement(actuatedDOFs, actuatedDOFs)
        , m_generalizedBiasForces(totalDOFs)
        , m_centroidalMomentum(6)
        , m_SBar(totalDOFs, actuatedDOFs)
        , m_feetJacobianTimesSBar(12, actuatedDOFs)
        , m_feetJacobianTimesSBarPseudoInverse(actuatedDOFs, 12)
        , m_centroidalForceMatrix(6, 12)
        , m_gravityForce(6)
        , m_desiredJointAcceleration1(actuatedDOFs)
        , m_desiredJointAcceleration2(actuatedDOFs)
        , m_UMatrix(6, totalDOFs)
        , m_rotoTranslationVector(7) {}
        
        TorqueBalancingController::~TorqueBalancingController() {}

#pragma mark - RateThread methods
        bool TorqueBalancingController::threadInit()
        {
            using namespace Eigen;
            //Initialize constant variables
            m_robot.getLinkId("l_sole", m_leftFootLinkID);
            m_robot.getLinkId("r_sole", m_rightFootLinkID);
            m_robot.getLinkId("com", m_centerOfMassLinkID);
            
            m_leftFootToBaseRotationFrame.R = wbi::Rotation(0, 0, 1,
                                                            0, -1, 0,
                                                            1, 0, 0);
            
            //centroidal force matrix
            m_centroidalForceMatrix.setZero();
            m_centroidalForceMatrix.block<3, 3>(0, 0) = Matrix3d::Identity();
            m_centroidalForceMatrix.block<3, 3>(0, 6) = Matrix3d::Identity();
            m_centroidalForceMatrix.block<3, 3>(3, 3) = Matrix3d::Identity();
            m_centroidalForceMatrix.block<3, 3>(3, 9) = Matrix3d::Identity();
            //gravity
            m_gravityForce.setZero();
            m_gravityUnitVector[0] = m_gravityUnitVector[1];
            //TODO: check sign of gravity
            m_gravityUnitVector[2] = -9.81;
            return true;
        }
        
        void TorqueBalancingController::threadRelease()
        {
            
        }
        
        void TorqueBalancingController::run()
        {
            if (!this->isActiveState()) return;
            //read references
            readReferences();
            
            //read / update state
            updateRobotState();
            
            //compute desired feet forces
            computeFeetForces(m_desiredCOMAcceleration, m_desiredFeetForces);
            
            //compute torques
            computeTorques(m_desiredFeetForces, m_torques);
            
            //write torques
            writeTorques();
            
        }
        
#pragma mark - Getter and setter
        
        double TorqueBalancingController::centroidalMomentumGain()
        {
            codyco::LockGuard guard(m_mutex);
            return m_centroidalMomentumGain;
        }
        
        void TorqueBalancingController::setCentroidalMomentumGain(double centroidalMomentumGain)
        {
            codyco::LockGuard guard(m_mutex);
            m_centroidalMomentumGain = centroidalMomentumGain;
        }
        
        void TorqueBalancingController::setActiveState(bool isActive)
        {
            codyco::LockGuard guard(m_mutex);
            if (m_active == isActive) return;
            if (isActive) {
                m_robot.setControlMode(wbi::CTRL_MODE_TORQUE);
            } else {
                m_robot.setControlMode(wbi::CTRL_MODE_POS);
            }
            m_active = isActive;
        }
        
        bool TorqueBalancingController::isActiveState()
        {
            codyco::LockGuard guard(m_mutex);
            return m_active;
        }
        
#pragma mark - Controller methods
        
        void TorqueBalancingController::readReferences()
        {
            if (m_references.desiredCOMAcceleration().isValid())
                m_desiredCOMAcceleration = m_references.desiredCOMAcceleration().value();
            //TODO: read other references
        }
        
        bool TorqueBalancingController::updateRobotState()
        {
            //update world to base frame
            m_robot.computeH(m_jointPositions.data(), wbi::Frame(), m_leftFootLinkID, m_world2BaseFrame);
            m_world2BaseFrame = m_world2BaseFrame * m_leftFootToBaseRotationFrame;
            m_world2BaseFrame.setToInverse();
            
            //read positions and velocities
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointPositions.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointVelocities.data());
            
            //update mass matrix
            m_robot.computeMassMatrix(m_jointPositions.data(), m_world2BaseFrame, m_massMatrix.data());
            m_robot.computeCentroidalMomentum(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_centroidalMomentum.data());

            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_centerOfMassLinkID, m_rotoTranslationVector.data());
            m_centerOfMassPosition = m_rotoTranslationVector.head<3>();
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_leftFootLinkID, m_leftFootPosition.data());
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_rightFootLinkID, m_rightFootPosition.data());
            
            return true;
        }
        
        void TorqueBalancingController::computeFeetForces(const Eigen::Ref<Eigen::MatrixXd>& desiredCOMAcceleration, Eigen::Ref<Eigen::MatrixXd> desiredFeetForces)
        {
            using namespace Eigen;
            double mass = m_massMatrix(0, 0);
            m_gravityForce(2) = -mass * 9.81;
            
            //building centroidalForceMatrix
            skewSymmentricMatrix(m_leftFootPosition.head<3>() - m_centerOfMassPosition, m_centroidalForceMatrix.block<3, 3>(3, 0));
            skewSymmentricMatrix(m_rightFootPosition.head<3>() - m_centerOfMassPosition, m_centroidalForceMatrix.block<3, 3>(3, 6));
            
            m_desiredCentroidalMomentum.head<3>() = mass * desiredCOMAcceleration;
            m_desiredCentroidalMomentum.tail<3>() = -centroidalMomentumGain() * m_centroidalMomentum.tail<3>();

            desiredFeetForces = m_centroidalForceMatrix.jacobiSvd(ComputeThinU | ComputeThinV).solve(m_desiredCentroidalMomentum - m_gravityForce);
            //TODO: we can also test LDLT decomposition since it requires positive semidefinite matrix
//            desiredFeetForces = m_centroidalForceMatrix.ldlt().solve(m_desiredCentroidalMomentum - m_gravityForce);
            
        }
        
        void TorqueBalancingController::computeTorques(const Eigen::Ref<Eigen::MatrixXd>& desiredFeetForces, Eigen::Ref<Eigen::MatrixXd> torques)
        {
            using namespace Eigen;
            m_inverseBaseMassMatrix = m_massMatrix.topLeftCorner<6, 6>().inverse().eval();
            m_massMatrixSchurComplement = m_massMatrix.block(6, 6, actuatedDOFs, actuatedDOFs) - m_massMatrix.block(6, 0, actuatedDOFs, 6) * m_inverseBaseMassMatrix * m_massMatrix.block(0, 6, 6, actuatedDOFs);

            //update jacobians (both feet in one variable)
            m_robot.computeJacobian(m_jointPositions.data(), m_world2BaseFrame, m_leftFootLinkID, m_feetJacobian.topRows(6).data());
            m_robot.computeJacobian(m_jointPositions.data(), m_world2BaseFrame, m_rightFootLinkID, m_feetJacobian.bottomRows(6).data());

            m_robot.computeDJdq(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_leftFootLinkID, m_feetDJacobianDq.topRows(6).data());
            m_robot.computeDJdq(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_rightFootLinkID, m_feetDJacobianDq.bottomRows(6).data());

            //update auxiliary variables
            //SBar(1:6, :) = - M(1:6, 1:6)^-1 * M(1:6, 7:end)
            m_SBar.block(0, 0, 6, actuatedDOFs) = -m_inverseBaseMassMatrix * m_massMatrix.block(0, 6, 6, actuatedDOFs);

            m_feetJacobianTimesSBar = m_feetJacobian * m_SBar;
            math::pseudoInverse(m_feetJacobianTimesSBar, PseudoInverseTolerance, m_feetJacobianTimesSBarPseudoInverse);
//
//            m_robot.computeGeneralizedBiasForces(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_gravityUnitVector, m_generalizedBiasForces.data());
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
        
        void TorqueBalancingController::writeTorques()
        {
            m_robot.setControlReference(m_torques.data());
        }
        
#pragma mark - Auxiliary functions
        
        void TorqueBalancingController::skewSymmentricMatrix(const Eigen::Ref<const Eigen::Vector3d>& vector, Eigen::Ref<Eigen::Matrix3d> skewSymmetricMatrix)
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
        
    }
}
