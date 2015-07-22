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
#include "DynamicConstraint.h"

#include <wbi/wholeBodyInterface.h>
#include <wbi/wbiUtil.h>
#include <codyco/MathUtils.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <codyco/Utils.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include <iostream>
#include <limits>

#include <Eigen/LU>

#define TORQUEBALANCING_STATEACTIVE_THRESHOLD 0.05

namespace codyco {
    namespace torquebalancing {

#pragma mark - Delegate classes definitions
        ControllerDelegate::~ControllerDelegate() {}
        void ControllerDelegate::controllerDidStart(TorqueBalancingController& controller) {}
        void ControllerDelegate::controllerDidStop(TorqueBalancingController& controller) {}

#pragma mark - Torque Balancing Controller Implementation

        TorqueBalancingController::TorqueBalancingController(int period, ControllerReferences& references, wbi::wholeBodyInterface& robot, int actuatedDOFs, double dynamicSmoothingTime)
        : RateThread(period)
        , m_robot(robot)
        , m_actuatedDOFs(actuatedDOFs)
        , m_dynamicsTransitionTime(dynamicSmoothingTime)
        , m_delegate(0)
        , m_active(false)
        , m_checkJointLimits(true)
        , m_centerOfMassLinkID(wbi::wholeBodyInterface::COM_LINK_ID)
        , m_references(references)
        , m_desiredJointsConfiguration(actuatedDOFs)
        , m_centroidalMomentumGain(0)
        , m_impedanceGains(actuatedDOFs)
        , m_desiredCOMAcceleration(3)
        , m_desiredFeetForces(12)
        , m_desiredCentroidalMomentum(6)
        , m_desiredContactForces(6 * 2)
        , m_jointPositions(actuatedDOFs)
        , m_jointVelocities(actuatedDOFs)
        , m_torques(actuatedDOFs)
        , m_baseVelocity(6)
        , m_world2BaseFrameSerialization(16)
        , m_centerOfMassPosition(3)
        , m_rightFootPosition(7)
        , m_leftFootPosition(7)
        , m_minJointLimits(actuatedDOFs)
        , m_maxJointLimits(actuatedDOFs)
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
        , m_nullSpaceProjectorOfJcMInvSt(actuatedDOFs, actuatedDOFs)
        //        , m_pseudoInverseOfJcBase(6, 12)
        , m_pseudoInverseOfCentroidalForceMatrix(12, 6)
        , m_nullSpaceOfCentroidalForceMatrix(12, 12)
        , m_pseudoInverseOfTauN0_f(12, actuatedDOFs)
        , m_svdDecompositionOfJcMInvSt(6 * 2, actuatedDOFs)
        , m_svdDecompositionOfJcBase(12, 6)
        , m_svdDecompositionOfCentroidalForceMatrix(6, 12)
        , m_svdDecompositionOfTauN0_f(actuatedDOFs, 12)
        , m_luDecompositionOfCentroidalMatrix(6)
        , m_rotoTranslationVector(7)
        , m_jointsZeroVector(actuatedDOFs)
        , m_esaZeroVector(6)
        , m_jacobianTemporary(6, actuatedDOFs + 6)
        , m_dJacobiaDqTemporary(6)
        , m_buffers(actuatedDOFs) {}

        TorqueBalancingController::Buffers::Buffers(int actuatedDOFs)
        : baseAndJointsVector(actuatedDOFs + 6)
        , jointsVector(actuatedDOFs)
        , jointsVector2(actuatedDOFs)
        , esaVector(6)
        , totalDoFsIdentity(Eigen::MatrixXd::Identity(actuatedDOFs + 6, actuatedDOFs + 6))
        , totalDoFsTimesTotalDoFs(actuatedDOFs + 6, actuatedDOFs + 6)
        , totalDoFsLDLTDecomposition(actuatedDOFs + 6)
        , twelveTimesTotalDoFs(12, actuatedDOFs + 6)
        , sixTimesSix(6, 6)
        , twelveTimesDoFs(12, actuatedDOFs)
        , dofsTimesSix(actuatedDOFs, 6)
        , dofsTimesDoFs(actuatedDOFs, actuatedDOFs)
        , twelveTimesTwelve(12, 12) {}

        TorqueBalancingController::~TorqueBalancingController() {}

#pragma mark - RateThread methods
        bool TorqueBalancingController::threadInit()
        {
            using namespace Eigen;
            //Initialize constant variables
            bool linkFound = true;
            linkFound = m_robot.getFrameList().idToIndex("l_sole", m_leftFootLinkID);
            linkFound = linkFound && m_robot.getFrameList().idToIndex("r_sole", m_rightFootLinkID);

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
            m_desiredContactForces.setZero();
            m_torques.setZero();

            //limits
            bool result = false;
            result = m_robot.getJointLimits(m_minJointLimits.data(), m_maxJointLimits.data());
            if (!result) {
                yError("Failed to compute joint limits.");
                return false;
            }

            if (this->m_checkJointLimits) {
                std::cout << "[INFO]Joint limits are:\nmin" <<m_minJointLimits.transpose() << "\nmax " << m_maxJointLimits.transpose() << "\n";
            } else {
                yInfo("Joint limits disabled");
            }

            //read the initial configuration
            int count = 10;

            do {
                result = m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointPositions.data());
                count--;
            } while(!result && count >0);

            std::stringstream formattedConstraintsString;
            formattedConstraintsString << m_activeConstraints.size() << " Dyn. Constraints = ";
            for (ConstraintsMap::const_iterator it = m_activeConstraints.begin();
                 it != m_activeConstraints.end(); it++) {
                formattedConstraintsString << it->first << " ";
            }
            yInfo("%s", formattedConstraintsString.str().c_str());


//            debugPort.open("/tb/debug:o");

            return linkFound && result && !m_activeConstraints.empty();
        }

        void TorqueBalancingController::threadRelease()
        {
            debugPort.close();
        }

        void TorqueBalancingController::run()
        {
            yarp::os::LockGuard guard(m_mutex);
            if (!m_active) return;

            //read references
            readReferences();

            //read / update state
            if (!updateRobotState()) {
                yInfo() << "Failed to update state. Deactivating control";
                m_robot.setControlMode(wbi::CTRL_MODE_POS);
                m_active = false;
                if (m_delegate) m_delegate->controllerDidStop(*this);
                return;
            }

            //Check limits
            if (m_checkJointLimits && !jointsInLimitRange()) {
                yInfo() << "Joint limits reached. Deactivating control";
                m_robot.setControlMode(wbi::CTRL_MODE_POS);
                m_active = false;
                if (m_delegate) m_delegate->controllerDidStop(*this);
                return;
            }

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
            yarp::os::LockGuard guard(m_mutex);
            return m_centroidalMomentumGain;
        }

        void TorqueBalancingController::setCentroidalMomentumGain(double centroidalMomentumGain)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_centroidalMomentumGain = centroidalMomentumGain;
        }

        const Eigen::VectorXd& TorqueBalancingController::impedanceGains()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_impedanceGains;
        }

        void TorqueBalancingController::setImpedanceGains(Eigen::VectorXd& impedanceGains)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_impedanceGains = impedanceGains;
        }

        void TorqueBalancingController::setActiveState(bool isActive)
        {
            if (m_active == isActive) return;

            yarp::os::LockGuard guard(m_mutex);
            if (isActive) {
                m_desiredCOMAcceleration.setZero(); //reset reference
                m_robot.setControlMode(wbi::CTRL_MODE_TORQUE);
            } else {
                std::cerr << "Deactivating control\n";
                m_robot.setControlMode(wbi::CTRL_MODE_POS);
            }
            m_active = isActive;
        }

        bool TorqueBalancingController::isActiveState()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_active;
        }
        
        void TorqueBalancingController::setCheckJointLimits(bool checkJointLimits)
        {
            yarp::os::LockGuard guard(m_mutex);
            this->m_checkJointLimits = checkJointLimits;
        }
            
        bool TorqueBalancingController::doesCheckJointLimits()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_checkJointLimits;
        }

        void TorqueBalancingController::setTorqueSaturationLimit(Eigen::VectorXd& newSaturation)
        {
            yarp::os::LockGuard guard(m_mutex);
            m_torqueSaturationLimit = newSaturation.array().abs();
        }

        const Eigen::VectorXd& TorqueBalancingController::torqueSaturationLimit()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_torqueSaturationLimit;
        }

        void TorqueBalancingController::setDelegate(ControllerDelegate *delegate)
        {
            yarp::os::LockGuard guard(m_mutex);
            this->m_delegate = delegate;
        }


        bool TorqueBalancingController::setInitialConstraintSet(const std::vector<std::string> &constraintsLinkName)
        {
            if (isRunning()) return false;
            //TODO: two days before the demo is not the right moment to make the constraints generic.
            // For now simply accept the l_sole or r_sole constraints.
            // move everything to something more generic ASAP
            bool result = true;
            m_activeConstraints.clear();
            for (std::vector<std::string>::const_iterator it = constraintsLinkName.begin();
                 it != constraintsLinkName.end(); it++) {
                if ((*it).compare("l_sole") == 0) {
                    DynamicConstraint leftFoot;
                    result = result && leftFoot.init(true, getRate() / 1000.0, m_dynamicsTransitionTime);
                    m_activeConstraints.insert(ConstraintsMap::value_type(*it, leftFoot));

                } else if ((*it).compare("r_sole") == 0) {
                    DynamicConstraint rightFoot;
                    result = result && rightFoot.init(true, getRate() / 1000.0, m_dynamicsTransitionTime);
                    m_activeConstraints.insert(ConstraintsMap::value_type(*it, rightFoot));
                }
            }

            return result && m_activeConstraints.size() >= 1 && m_activeConstraints.size() <= 2;
        }

        bool TorqueBalancingController::addDynamicConstraint(std::string frameName, bool /*smooth*/)
        {
            //For now full jacobians are not written in an "iterative" way.
            //just hardcode all the possible scenarios. (as in MATLAB version)
            //This must be changed in the future
            //            ConstraintsMap::iterator found = m_activeConstraints.find(frameName);
            //            if (found != m_activeConstraints.end()) return false;
            //            DynamicContraint constraint(frameName);
            //            constraint.createMemory();
            //            m_activeConstraints.insert(ConstraintsMap::value_type(frameName, constraint));
            yarp::os::LockGuard guard(m_mutex);
            ConstraintsMap::iterator found = m_activeConstraints.find(frameName);
            if (found == m_activeConstraints.end()) return false;
            found->second.activate();

            return true;
        }

        bool TorqueBalancingController::removeDynamicConstraint(std::string frameName, bool /*smooth*/)
        {
            yarp::os::LockGuard guard(m_mutex);
            ConstraintsMap::iterator found = m_activeConstraints.find(frameName);
            if (found == m_activeConstraints.end()) return false;
            found->second.deactivate();

            return true;
        }

#pragma mark - Monitorable variables

        const Eigen::VectorXd& TorqueBalancingController::desiredFeetForces()
        {
            yarp::os::LockGuard guard(m_mutex);
            return m_desiredFeetForces;
        }

        const Eigen::VectorXd& TorqueBalancingController::outputTorques()
        {
            yarp::os::LockGuard guard(m_mutex);
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
        }

        bool TorqueBalancingController::jointsInLimitRange()
        {
            for (int i = 0; i < m_jointPositions.size(); i++) {
                if (m_jointPositions(i) < m_minJointLimits(i) ||
                    m_jointPositions(i) > m_maxJointLimits(i)) {
                    yInfo("Joint %d is outside limit [%lf,%lf is %lf]. Stop control", i, m_minJointLimits(i), m_maxJointLimits(i), m_jointPositions(i));
                    return false;
                }
            }
            return true;
        }

        bool TorqueBalancingController::updateRobotState()
        {
            yarp::os::LockGuard guard(dynamic_cast<yarpWbi::yarpWholeBodyInterface*>(&m_robot)->getInterfaceMutex());
#if defined(DEBUG) && defined(EIGEN_RUNTIME_NO_MALLOC)
            Eigen::internal::set_is_malloc_allowed(false);
#endif
            bool result = true;
            //read positions and velocities
            result = result && m_robot.getEstimates(wbi::ESTIMATE_JOINT_POS, m_jointPositions.data());
            result = result && m_robot.getEstimates(wbi::ESTIMATE_JOINT_VEL, m_jointVelocities.data());

            result = result && m_robot.getEstimates(wbi::ESTIMATE_BASE_POS, m_world2BaseFrameSerialization.data());
            wbi::frameFromSerialization(m_world2BaseFrameSerialization.data(), m_world2BaseFrame);

            result = result && m_robot.getEstimates(wbi::ESTIMATE_BASE_VEL, m_baseVelocity.data());

            //update jacobians (both feet in one variable)
            ConstraintsMap::iterator leftFootConstraint = m_activeConstraints.find("l_sole");
            if (leftFootConstraint != m_activeConstraints.end()) {
                leftFootConstraint->second.updateStateInterpolation();
            }
            ConstraintsMap::iterator rightFootConstraint = m_activeConstraints.find("r_sole");
            if (rightFootConstraint != m_activeConstraints.end()) {
                rightFootConstraint->second.updateStateInterpolation();
            }

            m_contactsJacobian.setZero();
            if (leftFootConstraint != m_activeConstraints.end()
                && leftFootConstraint->second.isActiveWithThreshold(TORQUEBALANCING_STATEACTIVE_THRESHOLD)) {
                m_jacobianTemporary.setZero();
                m_robot.computeJacobian(m_jointPositions.data(), m_world2BaseFrame, m_leftFootLinkID, m_jacobianTemporary.data());
                m_jacobianTemporary *= leftFootConstraint->second.continuousValue();
                m_contactsJacobian.topRows(6) = m_jacobianTemporary;
            }
            if (rightFootConstraint != m_activeConstraints.end()
                && rightFootConstraint->second.isActiveWithThreshold(TORQUEBALANCING_STATEACTIVE_THRESHOLD)) {
                m_jacobianTemporary.setZero();
                m_robot.computeJacobian(m_jointPositions.data(), m_world2BaseFrame, m_rightFootLinkID, m_jacobianTemporary.data());
                m_jacobianTemporary *= rightFootConstraint->second.continuousValue();
                m_contactsJacobian.bottomRows(6) = m_jacobianTemporary;
            }

            //update kinematic quantities
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_centerOfMassLinkID, m_rotoTranslationVector.data());
            m_centerOfMassPosition = m_rotoTranslationVector.head<3>();
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_leftFootLinkID, m_leftFootPosition.data());
            m_robot.forwardKinematics(m_jointPositions.data(), m_world2BaseFrame, m_rightFootLinkID, m_rightFootPosition.data());
//            yarp::sig::Vector &v = debugPort.prepare();
//            v.resize(6+16);
//            v(0) = m_leftFootPosition(0) - m_centerOfMassPosition(0);
//            v(1) = m_leftFootPosition(1) - m_centerOfMassPosition(1);
//            v(2) = m_leftFootPosition(2) - m_centerOfMassPosition(2);
//            v(3) = m_rightFootPosition(0) - m_centerOfMassPosition(0);
//            v(4) = m_rightFootPosition(1) - m_centerOfMassPosition(1);
//            v(5) = m_rightFootPosition(2) - m_centerOfMassPosition(2);
//            for (int i = 0; i < 16; i++) {
//                v(6+ i) = m_world2BaseFrameSerialization.data()[i];
//            }
//            debugPort.write();

            //update dynamic quantities
            m_robot.computeMassMatrix(m_jointPositions.data(), m_world2BaseFrame, m_massMatrix.data());
            m_robot.computeCentroidalMomentum(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_centroidalMomentum.data());

            m_contactsDJacobianDq.setZero();
            if (leftFootConstraint != m_activeConstraints.end()
                && leftFootConstraint->second.isActiveWithThreshold(TORQUEBALANCING_STATEACTIVE_THRESHOLD)) {
                m_robot.computeDJdq(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_leftFootLinkID, m_contactsDJacobianDq.head(6).data());
                m_contactsDJacobianDq.head(6) *= leftFootConstraint->second.continuousValue();
            }
            if (rightFootConstraint != m_activeConstraints.end()
                && rightFootConstraint->second.isActiveWithThreshold(TORQUEBALANCING_STATEACTIVE_THRESHOLD)) {
                m_robot.computeDJdq(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_rightFootLinkID, m_contactsDJacobianDq.tail(6).data());
                m_contactsDJacobianDq *= rightFootConstraint->second.continuousValue();
            }

            //Compute bias forces
            m_robot.computeGeneralizedBiasForces(m_jointPositions.data(), m_world2BaseFrame, m_jointVelocities.data(), m_baseVelocity.data(), m_gravityUnitVector, m_generalizedBiasForces.data());
            m_robot.computeGeneralizedBiasForces(m_jointPositions.data(), m_world2BaseFrame, m_jointsZeroVector.data(), m_esaZeroVector.data(), m_gravityUnitVector, m_gravityBiasTorques.data());

#if defined(DEBUG) && defined(EIGEN_RUNTIME_NO_MALLOC)
            Eigen::internal::set_is_malloc_allowed(true);
#endif
            return result;
        }

        void TorqueBalancingController::computeContactForces(const Eigen::Ref<Eigen::VectorXd>& desiredCOMAcceleration, Eigen::Ref<Eigen::VectorXd> desiredContactForces)
        {
#if defined(DEBUG) && defined(EIGEN_RUNTIME_NO_MALLOC)
            Eigen::internal::set_is_malloc_allowed(false);
#endif
            using namespace Eigen;
            double mass = m_massMatrix(0, 0);
            m_gravityForce(2) = -mass * 9.81;

            ConstraintsMap::const_iterator leftFootConstraint = m_activeConstraints.find("l_sole");
            ConstraintsMap::const_iterator rightFootConstraint = m_activeConstraints.find("r_sole");

            //building centroidalForceMatrix
            bool leftConstraintIsActive = false;
            bool rightConstraintIsActive = false;

            m_centroidalForceMatrix.setZero();
            if (leftFootConstraint != m_activeConstraints.end()
                && leftFootConstraint->second.isActiveWithThreshold(TORQUEBALANCING_STATEACTIVE_THRESHOLD)) {
                leftConstraintIsActive = true;
                m_centroidalForceMatrix.block<3, 3>(0, 0) = Matrix3d::Identity();
                m_centroidalForceMatrix.block<3, 3>(3, 3) = Matrix3d::Identity();
                math::skewSymmentricMatrixFrom3DVector(m_leftFootPosition.head<3>() - m_centerOfMassPosition, m_centroidalForceMatrix.block<3, 3>(3, 0));
                m_centroidalForceMatrix.leftCols(6) *= leftFootConstraint->second.continuousValue();
            }

            if (rightFootConstraint != m_activeConstraints.end()
                && rightFootConstraint->second.isActiveWithThreshold(TORQUEBALANCING_STATEACTIVE_THRESHOLD)) {
                rightConstraintIsActive = true;
                m_centroidalForceMatrix.block<3, 3>(0, 6) = Matrix3d::Identity();
                m_centroidalForceMatrix.block<3, 3>(3, 9) = Matrix3d::Identity();
                math::skewSymmentricMatrixFrom3DVector(m_rightFootPosition.head<3>() - m_centerOfMassPosition, m_centroidalForceMatrix.block<3, 3>(3, 6));
                m_centroidalForceMatrix.rightCols(6) *= rightFootConstraint->second.continuousValue();
            }

            m_desiredCentroidalMomentum.head<3>() = mass * desiredCOMAcceleration;
            m_desiredCentroidalMomentum.tail<3>() = -m_centroidalMomentumGain * m_centroidalMomentum.tail<3>();

            //Eigen 3.3 will allow to set a threashold directly on the decomposition
            //thus allowing the method solve to work "properly".
            //Becaues it is not stable yet we use the explicit computation of the SVD
            //            m_svdDecompositionOfCentroidalForceMatrix.compute(m_centroidalForceMatrix).solve(m_desiredCentroidalMomentum - m_gravityForce);
            m_buffers.esaVector = m_desiredCentroidalMomentum - m_gravityForce;
            if (leftConstraintIsActive ^ rightConstraintIsActive) {
                m_desiredFeetForces.setZero();
                //substitute the pseudoinverse with its inverse
                m_luDecompositionOfCentroidalMatrix.compute(m_centroidalForceMatrix.block<6, 6>(0, leftConstraintIsActive ? 0 : 6));
                m_desiredFeetForces.segment(leftConstraintIsActive ? 0 : 6, 6) = m_luDecompositionOfCentroidalMatrix.solve(m_buffers.esaVector);
                m_nullSpaceOfCentroidalForceMatrix.setZero();

            } else {
                math::pseudoInverse(m_centroidalForceMatrix, m_svdDecompositionOfCentroidalForceMatrix,
                                    m_pseudoInverseOfCentroidalForceMatrix, PseudoInverseTolerance);
                m_desiredFeetForces.noalias() = (m_pseudoInverseOfCentroidalForceMatrix * m_buffers.esaVector);

                //TODO: change the following line by using the null space basis obtained by the pseudoinverse method
                m_nullSpaceOfCentroidalForceMatrix.setIdentity();
                m_nullSpaceOfCentroidalForceMatrix -= m_pseudoInverseOfCentroidalForceMatrix * m_centroidalForceMatrix;
            }
            desiredContactForces = m_desiredFeetForces;
#if defined(DEBUG) && defined(EIGEN_RUNTIME_NO_MALLOC)
            Eigen::internal::set_is_malloc_allowed(true);
#endif
        }

        void TorqueBalancingController::computeTorques(const Eigen::Ref<Eigen::VectorXd>& desiredContactForces, Eigen::Ref<Eigen::VectorXd> torques)
        {
            using namespace Eigen;

#if defined(DEBUG) && defined(EIGEN_RUNTIME_NO_MALLOC)
            Eigen::internal::set_is_malloc_allowed(false);
#endif

            m_buffers.totalDoFsLDLTDecomposition.compute(m_massMatrix);
            Eigen::internal::solve_retval<LDLT<MatrixXd::PlainObject>, MatrixXd> var =
            m_buffers.totalDoFsLDLTDecomposition.solve(m_buffers.totalDoFsIdentity);
            m_buffers.totalDoFsTimesTotalDoFs = var.rhs();

            m_buffers.twelveTimesTotalDoFs.noalias() = m_contactsJacobian * m_buffers.totalDoFsTimesTotalDoFs;
            m_buffers.twelveTimesTwelve.noalias() = m_buffers.twelveTimesTotalDoFs * m_contactsJacobian.transpose();
            m_buffers.sixTimesSix.noalias() = m_massMatrix.topLeftCorner<6, 6>().inverse();
            m_buffers.twelveTimesDoFs.noalias() = m_buffers.twelveTimesTotalDoFs * m_torquesSelector;
            m_buffers.dofsTimesSix.noalias() = m_massMatrix.block(6, 0, m_actuatedDOFs, 6) * m_buffers.sixTimesSix; //* m_massMatrix.topLeftCorner<6, 6>().inverse();

            math::dampedPseudoInverse(m_buffers.twelveTimesDoFs, m_svdDecompositionOfJcMInvSt, m_pseudoInverseOfJcMInvSt,
                                      PseudoInverseTolerance,
                                      JcMInvSPseudoInverseDampingTerm);
            math::pseudoInverse(m_buffers.twelveTimesDoFs, m_svdDecompositionOfJcMInvSt,
                                m_pseudoInverseOfJcMInvSt, PseudoInverseTolerance);
            //TODO: change the following line by using the null space basis obtained by the pseudoinverse method

            m_nullSpaceProjectorOfJcMInvSt.setIdentity();
            m_nullSpaceProjectorOfJcMInvSt/*.noalias()*/ -= m_pseudoInverseOfJcMInvSt * m_buffers.twelveTimesDoFs;


            MatrixXd mult_f_tau0 =  m_buffers.dofsTimesSix * m_contactsJacobian.leftCols(6).transpose() - m_contactsJacobian.rightCols(m_actuatedDOFs).transpose();

            m_buffers.jointsVector =  m_gravityBiasTorques.tail(m_actuatedDOFs) - m_impedanceGains.asDiagonal() * (m_jointPositions - m_desiredJointsConfiguration) - m_buffers.dofsTimesSix * m_generalizedBiasForces.head<6>();

            MatrixXd mult_f_tau = -m_pseudoInverseOfJcMInvSt * m_buffers.twelveTimesTwelve + m_nullSpaceProjectorOfJcMInvSt * mult_f_tau0;

            m_buffers.jointsVector2 = m_pseudoInverseOfJcMInvSt * m_buffers.twelveTimesTotalDoFs * m_generalizedBiasForces;
            m_buffers.jointsVector2 -= m_pseudoInverseOfJcMInvSt * m_contactsDJacobianDq;
            m_buffers.jointsVector2 += m_nullSpaceProjectorOfJcMInvSt * m_buffers.jointsVector;

            math::pseudoInverse(mult_f_tau * m_nullSpaceOfCentroidalForceMatrix, m_svdDecompositionOfTauN0_f,m_pseudoInverseOfTauN0_f, PseudoInverseTolerance, Eigen::ComputeFullV|Eigen::ComputeFullU);

            m_buffers.dofsTimesDoFs.setIdentity();

            torques = (m_buffers.dofsTimesDoFs - mult_f_tau * m_nullSpaceOfCentroidalForceMatrix * m_pseudoInverseOfTauN0_f) * (m_buffers.jointsVector2 + mult_f_tau * desiredContactForces);

            //apply saturation
            //TODO: this must be checked: valgrind says it contains a jump on an unitialized variable
            //TODO: check isinf or isnan
            torques = torques.array().min(m_torqueSaturationLimit.array()).max(-m_torqueSaturationLimit.array());
#if defined(DEBUG) && defined(EIGEN_RUNTIME_NO_MALLOC)
            Eigen::internal::set_is_malloc_allowed(true);
#endif
        }

        void TorqueBalancingController::writeTorques()
        {
            m_robot.setControlReference(m_torques.data());
        }

#pragma mark - Auxiliary functions

    }
}
