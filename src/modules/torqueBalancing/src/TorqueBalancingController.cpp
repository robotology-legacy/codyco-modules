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
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <codyco/Utils.h>
#include <iostream>
#include <limits>

#include <Eigen/LU>


namespace codyco {
    namespace torquebalancing {

        TorqueBalancingController::TorqueBalancingController(int period, ControllerReferences& references, wbi::wholeBodyInterface& robot, int actuatedDOFs)
        : RateThread(period)
        , m_robot(robot)
        , m_actuatedDOFs(actuatedDOFs)
        , m_active(false)
        , m_centerOfMassLinkID(wbi::wholeBodyInterface::COM_LINK_ID)
        , m_references(references)
        , m_desiredJointsConfiguration(actuatedDOFs)
        , m_centroidalMomentumGain(0)
        , m_impedanceGains(actuatedDOFs)
        , m_desiredCOMAcceleration(3)
        , m_desiredFeetForces(12)
        , m_desiredCentroidalMomentum(6)
        , m_desiredHandsForces(12)
        , m_desiredContactForces(6 * 2)
        , m_leftHandForcesActive(false)
        , m_rightHandForcesActive(false)
        , m_jointPositions(actuatedDOFs)
        , m_jointVelocities(actuatedDOFs)
        , m_torques(actuatedDOFs)
        , m_baseVelocity(6)
        , m_centerOfMassPosition(3)
        , m_rightFootPosition(7)
        , m_leftFootPosition(7)
        , m_torqueSaturationLimit(actuatedDOFs)
        , m_contactsJacobian(6 * 2, actuatedDOFs + 6)
        , m_contactsDJacobianDq(6 * 2)
        , m_massMatrix(actuatedDOFs + 6, actuatedDOFs + 6)
        , m_generalizedBiasForces(actuatedDOFs + 6)
        , m_gravityBiasTorques(actuatedDOFs + 6)
        , m_centroidalMomentum(6)
        , m_centroidalForceMatrix(6, 12)
        , m_gravityForce(6)
        , m_torquesSelector(actuatedDOFs + 6, actuatedDOFs)
        , m_pseudoInverseOfJcMInvSt(actuatedDOFs, 6 * 2)
        , m_pseudoInverseOfJcBase(6, 12)
        , m_pseudoInverseOfCentroidalForceMatrix(12, 6)
        , m_pseudoInverseOfTauN0_f(12, actuatedDOFs)
        , m_svdDecompositionOfJcMInvSt(6 * 2, actuatedDOFs)
        , m_svdDecompositionOfJcBase(12, 6)
        , m_svdDecompositionOfCentroidalForceMatrix(6, 12)
        , m_svdDecompositionOfTauN0_f(actuatedDOFs, 12)
        , m_rotoTranslationVector(7)
        , m_jointsZeroVector(actuatedDOFs)
        , m_esaZeroVector(6)
        , m_jacobianTemporary(6, actuatedDOFs + 6)
        , m_dJacobiaDqTemporary(6) {}

        TorqueBalancingController::~TorqueBalancingController() {}

#pragma mark - RateThread methods
        bool TorqueBalancingController::threadInit()
        {
            using namespace Eigen;
            //Initialize constant variables
            bool linkFound = true;
            linkFound = m_robot.getFrameList().idToIndex("l_sole", m_leftFootLinkID);
            linkFound = linkFound && m_robot.getFrameList().idToIndex("r_sole", m_rightFootLinkID);
            //linkFound = linkFound && m_robot.getFrameList().idToIndex("l_gripper", m_leftHandLinkID);
            //linkFound = linkFound && m_robot.getFrameList().idToIndex("r_gripper", m_rightHandLinkID);

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
            m_gravityUnitVector[0] = m_gravityUnitVector[1] = 0;
            m_gravityUnitVector[2] = -9.81;

            m_torquesSelector.setZero();
            m_torquesSelector.bottomRows(m_actuatedDOFs).setIdentity();

            m_jointsZeroVector.setZero();
            m_esaZeroVector.setZero();
            m_torqueSaturationLimit.setConstant(std::numeric_limits<double>::max());

            //reset status to zero
            m_jointPositions.setZero();
            m_jointVelocities.setZero();
            m_baseVelocity.setZero();
            m_centerOfMassPosition.setZero();
            m_rightFootPosition.setZero();
            m_leftFootPosition.setZero();
            m_contactsJacobian.setZero();
            m_contactsDJacobianDq.setZero();
            m_generalizedBiasForces.setZero();
            m_gravityBiasTorques.setZero();
            m_centroidalMomentum.setZero();
            m_massMatrix.setZero();

            m_desiredJointsConfiguration.setZero();

            //zeroing gains
            m_impedanceGains.setZero();

            //zeroing monitored variables
            m_desiredFeetForces.setZero();
            m_desiredHandsForces.setZero();
            m_desiredContactForces.setZero();
            m_torques.setZero();
            return linkFound;
        }

        void TorqueBalancingController::threadRelease()
        {

        }

        void TorqueBalancingController::run()
        {
            codyco::LockGuard guard(m_mutex);
            if (!m_active) return;

            //read references
            readReferences();

            //read / update state
            updateRobotState();

            //compute desired feet forces
            computeContactForces(m_desiredCOMAcceleration, m_desiredContactForces);

            //compute torques
            computeTorques(m_desiredContactForces, m_torques);

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

        const Eigen::VectorXd& TorqueBalancingController::impedanceGains()
        {
            codyco::LockGuard guard(m_mutex);
            return m_impedanceGains;
        }

        void TorqueBalancingController::setImpedanceGains(Eigen::VectorXd& impedanceGains)
        {
            codyco::LockGuard guard(m_mutex);
            m_impedanceGains = impedanceGains;
        }

        void TorqueBalancingController::setActiveState(bool isActive)
        {
            codyco::LockGuard guard(m_mutex);
            if (m_active == isActive) return;
            if (isActive) {
                m_desiredCOMAcceleration.setZero(); //reset reference
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

        void TorqueBalancingController::setTorqueSaturationLimit(Eigen::VectorXd& newSaturation)
        {
            codyco::LockGuard guard(m_mutex);
            m_torqueSaturationLimit = newSaturation.array().abs();
        }

        const Eigen::VectorXd& TorqueBalancingController::torqueSaturationLimit()
        {
            codyco::LockGuard guard(m_mutex);
            return m_torqueSaturationLimit;
        }

#pragma mark - Monitorable variables

        const Eigen::VectorXd& TorqueBalancingController::desiredFeetForces()
        {
            codyco::LockGuard guard(m_mutex);
            return m_desiredFeetForces;
        }

        const Eigen::VectorXd& TorqueBalancingController::outputTorques()
        {
            codyco::LockGuard guard(m_mutex);
            return m_torques;
        }

#pragma mark - Controller methods

        void TorqueBalancingController::readReferences()
        {
            if (m_references.desiredCOMAcceleration().isValid())
                m_desiredCOMAcceleration = m_references.desiredCOMAcceleration().value();
            if (m_references.desiredJointsConfiguration().isValid()) {
                m_desiredJointsConfiguration = m_references.desiredJointsConfiguration().value();
            }
//             if ((m_leftHandForcesActive = m_references.desiredLeftHandForce().isValid()))
//                 m_desiredHandsForces.head(6) = m_references.desiredLeftHandForce().value();
//             if ((m_rightHandForcesActive = m_references.desiredRightHandForce().isValid()))
//                 m_desiredHandsForces.tail(6) = m_references.desiredRightHandForce().value();
        }

        bool TorqueBalancingController::updateRobotState()
        {
            codyco::LockGuard guard(dynamic_cast<yarpWbi::yarpWholeBodyInterface*>(&m_robot)->getInterfaceMutex());
            //read positions and velocities
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointPositions.data());
            m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointVelocities.data());

            //update world to base frame
            m_robot.computeH(m_jointPositions.data(), wbi::Frame(), m_leftFootLinkID, m_world2BaseFrame);
            m_world2BaseFrame = m_world2BaseFrame * m_leftFootToBaseRotationFrame;
            m_world2BaseFrame.setToInverse();

            //update jacobians (both feet in one variable)
            m_contactsJacobian.setZero();
            m_jacobianTemporary.setZero();
            m_robot.computeJacobian(m_jointPositions.data(), m_world2BaseFrame, m_leftFootLinkID, m_jacobianTemporary.data());
            m_contactsJacobian.topRows(6) = m_jacobianTemporary;
            m_jacobianTemporary.setZero();
            m_robot.computeJacobian(m_jointPositions.data(), m_world2BaseFrame, m_rightFootLinkID, m_jacobianTemporary.data());
            m_contactsJacobian.bottomRows(6) = m_jacobianTemporary;

            //update kinematic quantities
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_centerOfMassLinkID, m_rotoTranslationVector.data());
            m_centerOfMassPosition = m_rotoTranslationVector.head<3>();
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_leftFootLinkID, m_leftFootPosition.data());
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_rightFootLinkID, m_rightFootPosition.data());

            //update base velocity (to be moved in wbi state)
            math::pseudoInverse(m_contactsJacobian.topLeftCorner<12, 6>(), m_svdDecompositionOfJcBase,
                                m_pseudoInverseOfJcBase, PseudoInverseTolerance);
            m_baseVelocity = -m_pseudoInverseOfJcBase * m_contactsJacobian.topRightCorner(12, m_actuatedDOFs) * m_jointVelocities;

            //update dynamic quantities
            m_robot.computeMassMatrix(m_jointPositions.data(), m_world2BaseFrame, m_massMatrix.data());
            m_robot.computeCentroidalMomentum(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_centroidalMomentum.data());

            m_contactsDJacobianDq.setZero();
            m_robot.computeDJdq(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_leftFootLinkID, m_contactsDJacobianDq.head(6).data());
            m_robot.computeDJdq(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_rightFootLinkID, m_contactsDJacobianDq.tail(6).data());

            //Compute bias forces
            m_robot.computeGeneralizedBiasForces(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_gravityUnitVector, m_generalizedBiasForces.data());
            m_robot.computeGeneralizedBiasForces(m_jointPositions.data(), m_world2BaseFrame, m_jointsZeroVector.data(), m_esaZeroVector.data(), m_gravityUnitVector, m_gravityBiasTorques.data());

            return true;
        }

        void TorqueBalancingController::computeContactForces(const Eigen::Ref<Eigen::MatrixXd>& desiredCOMAcceleration, Eigen::Ref<Eigen::MatrixXd> desiredContactForces)
        {
            using namespace Eigen;
            double mass = m_massMatrix(0, 0);
            m_gravityForce(2) = -mass * 9.81;

            //building centroidalForceMatrix
            math::skewSymmentricMatrixFrom3DVector(m_leftFootPosition.head<3>() - m_centerOfMassPosition, m_centroidalForceMatrix.block<3, 3>(3, 0));
            math::skewSymmentricMatrixFrom3DVector(m_rightFootPosition.head<3>() - m_centerOfMassPosition, m_centroidalForceMatrix.block<3, 3>(3, 6));

            m_desiredCentroidalMomentum.head<3>() = mass * desiredCOMAcceleration;
            m_desiredCentroidalMomentum.tail<3>() = -m_centroidalMomentumGain * m_centroidalMomentum.tail<3>();

            //Eigen 3.3 will allow to set a threashold directly on the decomposition
            //thus allowing the method solve to work "properly".
            //Becaues it is not stable yet we use the explicit computation of the SVD
//            m_svdDecompositionOfCentroidalForceMatrix.compute(m_centroidalForceMatrix).solve(m_desiredCentroidalMomentum - m_gravityForce);

            math::pseudoInverse(m_centroidalForceMatrix, m_svdDecompositionOfCentroidalForceMatrix,
                                m_pseudoInverseOfCentroidalForceMatrix, PseudoInverseTolerance);

            m_desiredFeetForces = m_pseudoInverseOfCentroidalForceMatrix * (m_desiredCentroidalMomentum
                                                                            - m_gravityForce
                                                                            - m_desiredHandsForces.head(6) - m_desiredHandsForces.tail(6));

            desiredContactForces = m_desiredFeetForces;
        }

        void TorqueBalancingController::computeTorques(const Eigen::Ref<Eigen::VectorXd>& desiredContactForces, Eigen::Ref<Eigen::MatrixXd> torques)
        {
            using namespace Eigen;

            //TODO: decide later if there is a performance benefit in moving the declaration of the variables in the class (or if this becomes a "new" at runtime)
            //Names are taken from "math" from brevity
            MatrixXd JcMInv = m_contactsJacobian * m_massMatrix.inverse(); //to become instance (?)
            MatrixXd JcMInvJct = JcMInv * m_contactsJacobian.transpose(); //to become instance (?)
            MatrixXd JcMInvTorqueSelector = JcMInv * m_torquesSelector; //to become instance (?)
            MatrixXd jointProjectedBaseAccelerations = m_massMatrix.block(6, 0, m_actuatedDOFs, 6) * m_massMatrix.topLeftCorner<6, 6>().inverse();

            math::pseudoInverse(JcMInvTorqueSelector, m_svdDecompositionOfJcMInvSt,
                                m_pseudoInverseOfJcMInvSt, PseudoInverseTolerance);
            MatrixXd JcNullSpaceProjector = MatrixXd::Identity(m_actuatedDOFs, m_actuatedDOFs) - m_pseudoInverseOfJcMInvSt * JcMInvTorqueSelector;

            MatrixXd mult_f_tau0 =  jointProjectedBaseAccelerations * m_contactsJacobian.leftCols(6).transpose() - m_contactsJacobian.rightCols(m_actuatedDOFs).transpose();

            VectorXd  torques0 =  m_gravityBiasTorques.tail(m_actuatedDOFs) - m_impedanceGains.asDiagonal() * (m_jointPositions - m_desiredJointsConfiguration) - jointProjectedBaseAccelerations * m_generalizedBiasForces.head<6>();

            MatrixXd mult_f_tau = -m_pseudoInverseOfJcMInvSt * JcMInvJct + JcNullSpaceProjector * mult_f_tau0;

            VectorXd n_tau = m_pseudoInverseOfJcMInvSt * (JcMInv * m_generalizedBiasForces - m_contactsDJacobianDq) + JcNullSpaceProjector * torques0;

            MatrixXd forceMatrixNullSpaceProjector = MatrixXd::Identity(12, 12) - m_pseudoInverseOfCentroidalForceMatrix * m_centroidalForceMatrix;

            MatrixXd mat = mult_f_tau * forceMatrixNullSpaceProjector;

            math::pseudoInverse(mult_f_tau * forceMatrixNullSpaceProjector, m_svdDecompositionOfTauN0_f,m_pseudoInverseOfTauN0_f, PseudoInverseTolerance);

            torques = (MatrixXd::Identity(n_tau.size(), n_tau.size()) -mult_f_tau * forceMatrixNullSpaceProjector * m_pseudoInverseOfTauN0_f) * (n_tau + mult_f_tau * desiredContactForces);


            //apply saturation
            //TODO: this must be checked: valgrind says it contains a jump on an unitialized variable
            //TODO: check isinf or isnan
            torques = torques.array().min(m_torqueSaturationLimit.array()).max(-m_torqueSaturationLimit.array());
        }

        void TorqueBalancingController::writeTorques()
        {
            m_robot.setControlReference(m_torques.data());
        }

#pragma mark - Auxiliary functions

    }
}
